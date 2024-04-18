## Build
For building on RasPi Zero, you need to do [**cross-compilation**](docs/CrossCompilation.md).

After it was built the first time, next builds are done as simple as:
```
docker compose run --rm main

$ pibuild --packages-up-to rumicar
```
