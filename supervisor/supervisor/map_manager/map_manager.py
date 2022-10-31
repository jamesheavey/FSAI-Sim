from ..setup import Setup


class MapManager:
    def __init__(self, ros, supervisor):
        self._ros = ros
        self._supervisor = supervisor
        # TODO: account for track width and params in cache
        # TODO: Clear item from cache when deleted (or updated)
        self._cache = {}
        self._cacheable_types = ("static", "optimumlap")

    def _get_cache_key(self, name, params={}):
        return (
            name,
            params.get("track_width"),
            params.get("fuzzing", {}).get("coneMisplacement"),
            params.get("fuzzing", {}).get("coneMissing"),
            params.get("fuzzing", {}).get("seed"),
        )

    def get_map(self, name, params={}):
        map_type = self._supervisor.options.get_map_type(name)
        track_width = params.get("track_width", 0)

        if (cached := self._cache.get(map_type, {}).get(self._get_cache_key(name, params), None)):
            return cached

        if map_type == "static":
            result = self._ros.service_request_blocking(
                Setup.Services.SimulatorManager.get_static_map,
                name, track_width
            )
        elif map_type == "optimumlap":
            result = self._ros.service_request_blocking(
                Setup.Services.SimulatorManager.get_optimumlap_map,
                self._supervisor.options.optimumlap_files.get_full_path(name),
                track_width
            )
        elif map_type == "random":
            result = self._ros.service_request_blocking(
                Setup.Services.SimulatorManager.get_random_map,
                params,
                track_width
            )
        elif map_type == "semi_static":
            result = self._ros.service_request_blocking(
                Setup.Services.SimulatorManager.get_semi_static_map,
                name,
                **params
            )
        else:
            raise LookupError(f"Unsupported map type {map_type}")

        if not result.success:
            raise Exception("Unsuccessfull map request")

        generate_map = result.map

        fuzzing = params.get("fuzzing", {})
        if fuzzing.get("coneMisplacement", 0) != 0 or fuzzing.get("coneMissing", 0) != 0:
            fuzzed_result = self._ros.service_request_blocking(
                Setup.Services.SimulatorManager.get_fuzzed_map,
                generate_map,
                fuzzing.get("seed", 0),
                fuzzing.get("coneMisplacement", 0),
                fuzzing.get("coneMissing", 0)
            )
            if fuzzed_result.success:
                generate_map = fuzzed_result.map

        if map_type in self._cacheable_types:
            if map_type not in self._cache:
                self._cache[map_type] = {}
            name = self._get_cache_key(name, params)
            self._cache[map_type][name] = generate_map

        return generate_map
