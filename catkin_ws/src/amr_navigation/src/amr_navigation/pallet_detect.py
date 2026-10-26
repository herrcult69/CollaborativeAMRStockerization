"""
Load-face pallet detection from a 2D laser scan.

The front lidar (~0.13 m) passes over the pallet blocks and hits the load box. The box front face
is a straight segment of known width centred on the pallet, so fitting a line to it gives the
pallet centre and heading in the scan's target frame (odom), independent of AMCL.
Pure functions only: any sensor that yields 2D points can feed detect_load_face().

Rack-leg detection (detect_rack_legs): with the lidar lowered under the bottom plank (~0.04 m), the scan hits
the two floor runners flanking the bay instead, which works for full and empty bays alike.
"""
import math


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max, tx, ty, tyaw):
    """Laser ranges -> list of (x, y) in the frame where the laser sits at (tx, ty, tyaw rad)."""
    c, s = math.cos(tyaw), math.sin(tyaw)
    pts = []
    for i, r in enumerate(ranges):
        if not (range_min < r < range_max) or math.isinf(r) or math.isnan(r):
            continue
        a = angle_min + i * angle_increment
        lx, ly = r * math.cos(a), r * math.sin(a)
        pts.append((tx + c * lx - s * ly, ty + s * lx + c * ly))
    return pts


def _fit_line(pts):
    """Least-squares line through pts: (centroid, unit direction, rms perpendicular residual)."""
    n = float(len(pts))
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    syy = sum((p[1] - my) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    ang = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    dx, dy = math.cos(ang), math.sin(ang)
    res = [-(p[0] - mx) * dy + (p[1] - my) * dx for p in pts]
    rms = math.sqrt(sum(r * r for r in res) / n)
    return (mx, my), (dx, dy), res, rms


def detect_load_face(points, exp_x, exp_y, exp_dock_yaw_deg, width=0.35, face_offset=0.15,
                     roi_lateral=0.30, roi_depth=0.20, width_tol=0.05, outlier=0.015, min_gap=0.03,
                     max_rms=0.01, max_offset=0.10, max_yaw_deg=5.0, min_points=10):
    """
    Find the load face of the pallet expected at (exp_x, exp_y), docked with heading exp_dock_yaw_deg
    (the robot heading while reversing in; it points from the pallet toward the robot at the stage).
    Neighbouring loads in the rack row lie on the same line; the face is split at gaps > min_gap
    (rays through a gap hit the rack behind and drop out) and the segment nearest the expectation is kept.

    :return: ((pallet_x, pallet_y, dock_yaw_deg), None) on success, or (None, reason) on failure.
    """
    ux, uy = math.cos(math.radians(exp_dock_yaw_deg)), math.sin(math.radians(exp_dock_yaw_deg))
    vx, vy = -uy, ux
    fx, fy = exp_x + face_offset * ux, exp_y + face_offset * uy  # expected face centre

    roi = [p for p in points
           if abs((p[0] - fx) * vx + (p[1] - fy) * vy) <= roi_lateral
           and abs((p[0] - fx) * ux + (p[1] - fy) * uy) <= roi_depth]
    if len(roi) < min_points:
        return None, "only %d scan points near the expected load face" % len(roi)

    # Seed on the median depth: the face holds most ROI points, while the side walls of neighbouring loads
    # (rays into the gaps) would pull a plain least-squares fit ~2 cm behind the face.
    depth = lambda p: (p[0] - fx) * ux + (p[1] - fy) * uy
    med = sorted(depth(p) for p in roi)[len(roi) // 2]
    pts = [p for p in roi if abs(depth(p) - med) <= 0.03]  # 0.03 m covers a 5 deg tilt across the ROI
    if len(pts) < min_points:
        return None, "only %d points at the load face depth" % len(pts)
    for _ in range(2):  # fit, drop points off the face (rack posts, background), refit
        _, _, res, _ = _fit_line(pts)
        kept = [p for p, r in zip(pts, res) if abs(r) <= outlier]
        if len(kept) < min_points:
            return None, "only %d points on a straight face" % len(kept)
        pts = kept

    # Split the line at gaps between neighbouring loads; keep the segment closest to the expected face
    (cx, cy), (dx, dy), _, _ = _fit_line(pts)
    pts = sorted(pts, key=lambda p: (p[0] - cx) * dx + (p[1] - cy) * dy)
    segments, seg = [], [pts[0]]
    for a, b in zip(pts, pts[1:]):
        if (b[0] - a[0]) * dx + (b[1] - a[1]) * dy > min_gap:
            segments.append(seg)
            seg = []
        seg.append(b)
    segments.append(seg)

    def seg_dist(s):
        mx, my = 0.5 * (s[0][0] + s[-1][0]), 0.5 * (s[0][1] + s[-1][1])
        return abs((mx - fx) * vx + (my - fy) * vy)

    pts = min(segments, key=seg_dist)
    if len(pts) < min_points:
        return None, "only %d points on the load face segment" % len(pts)
    (cx, cy), (dx, dy), _, rms = _fit_line(pts)
    if rms > max_rms:
        return None, "face not straight (rms %.1f mm)" % (rms * 1000.0)

    along = [(p[0] - cx) * dx + (p[1] - cy) * dy for p in pts]
    lo, hi = min(along), max(along)
    seen = hi - lo
    if abs(seen - width) > width_tol:
        return None, "face width %.3f m, expected %.3f m" % (seen, width)

    # Face normal pointing toward the robot side (same sense as the dock heading)
    nx, ny = -dy, dx
    if nx * ux + ny * uy < 0.0:
        nx, ny = -nx, -ny
    mid = 0.5 * (lo + hi)
    face_x, face_y = cx + mid * dx, cy + mid * dy
    px, py = face_x - face_offset * nx, face_y - face_offset * ny
    yaw_deg = math.degrees(math.atan2(ny, nx))

    off = math.hypot(px - exp_x, py - exp_y)
    lat = (px - exp_x) * vx + (py - exp_y) * vy    # + = left of the dock line, seen from the robot reversing in
    depth = (px - exp_x) * ux + (py - exp_y) * uy  # + = toward the robot (shallower)
    dyaw = (yaw_deg - exp_dock_yaw_deg + 180.0) % 360.0 - 180.0
    if off > max_offset or abs(dyaw) > max_yaw_deg:
        return None, "detected pallet %.3f m (lateral %+.3f, depth %+.3f) / %+.1f deg from expected" % (
            off, lat, depth, dyaw)
    return (px, py, exp_dock_yaw_deg + dyaw), None


def detect_rack_legs(points, exp_x, exp_y, exp_dock_yaw_deg, span=0.42, leg_width=0.08, leg_offset=0.19,
                     roi_lateral=0.36, roi_depth=0.15, min_gap=0.03, front_band=0.012, width_tol=0.03,
                     span_tol=0.03, max_offset=0.10, max_yaw_deg=5.0, min_points=4):
    """
    Find the bay of the pallet expected at (exp_x, exp_y) from the two floor runners ("legs") flanking it
    (Rack-DetectReady2: 0.08 m wide, 0.42 m pitch, under the bottom plank). The laser scans below the plank,
    so the signature is the same whether the bay holds a pallet or is empty.
    leg_offset: runner front faces, measured from the pallet centre toward the aisle.
    Same frame and return convention as detect_load_face().

    :return: ((pallet_x, pallet_y, dock_yaw_deg), None) on success, or (None, reason) on failure.
    """
    ux, uy = math.cos(math.radians(exp_dock_yaw_deg)), math.sin(math.radians(exp_dock_yaw_deg))
    vx, vy = -uy, ux
    fx, fy = exp_x + leg_offset * ux, exp_y + leg_offset * uy  # expected centre between the front faces
    lat = lambda p: (p[0] - fx) * vx + (p[1] - fy) * vy
    depth = lambda p: (p[0] - fx) * ux + (p[1] - fy) * uy     # + = toward the robot

    roi = sorted((p for p in points if abs(lat(p)) <= roi_lateral and abs(depth(p)) <= roi_depth), key=lat)
    if len(roi) < 2 * min_points:
        return None, "only %d scan points near the expected rack legs" % len(roi)

    # Rays through the bay throat pass under the plank and leave the ROI, so runners are split by lateral gaps
    clusters, cl = [], [roi[0]]
    for a, b in zip(roi, roi[1:]):
        if lat(b) - lat(a) > min_gap:
            clusters.append(cl)
            cl = []
        cl.append(b)
    clusters.append(cl)

    # Keep only each runner's front face: its 0.40 m long sides add points along the inner edge only
    legs = []
    for cl in clusters:
        front = max(depth(p) for p in cl)
        face = [p for p in cl if depth(p) >= front - front_band]
        if len(face) < min_points:
            continue
        lo, hi = lat(face[0]), lat(face[-1])
        if abs((hi - lo) - leg_width) > width_tol:
            continue
        d = sum(depth(p) for p in face) / len(face)
        m = 0.5 * (lo + hi)
        legs.append((fx + m * vx + d * ux, fy + m * vy + d * uy, m))
    pairs = [(a, b) for i, a in enumerate(legs) for b in legs[i + 1:]
             if abs(math.hypot(b[0] - a[0], b[1] - a[1]) - span) <= span_tol]
    if not pairs:
        return None, "no leg pair %.2f m apart among %d leg candidates" % (span, len(legs))
    left, right = min(pairs, key=lambda ab: abs(ab[0][2] + ab[1][2]))  # pair centred closest to expected

    # Bay normal toward the aisle: perpendicular to the leg-to-leg line, same sense as the dock heading
    dx, dy = right[0] - left[0], right[1] - left[1]
    nx, ny = -dy, dx
    if nx * ux + ny * uy < 0.0:
        nx, ny = -nx, -ny
    n = math.hypot(nx, ny)
    nx, ny = nx / n, ny / n
    px = 0.5 * (left[0] + right[0]) - leg_offset * nx
    py = 0.5 * (left[1] + right[1]) - leg_offset * ny
    yaw_deg = math.degrees(math.atan2(ny, nx))

    off = math.hypot(px - exp_x, py - exp_y)
    dyaw = (yaw_deg - exp_dock_yaw_deg + 180.0) % 360.0 - 180.0
    if off > max_offset or abs(dyaw) > max_yaw_deg:
        return None, "detected bay %.3f m (lateral %+.3f, depth %+.3f) / %+.1f deg from expected" % (
            off, (px - exp_x) * vx + (py - exp_y) * vy, (px - exp_x) * ux + (py - exp_y) * uy, dyaw)
    return (px, py, exp_dock_yaw_deg + dyaw), None
