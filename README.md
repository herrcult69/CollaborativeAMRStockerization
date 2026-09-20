# CollaborativeAMRStockerization


## Specify and Start container

Since there are two different docker-compose files for Ubuntu and Windows, please choose the correct one according to your OS.

```bash
docker compose -f docker-compose.[ubuntu/windows].yml build
docker compose -f docker-compose.[ubuntu/windows].yml up -d
docker exec -it ros1_amr_core bash
```