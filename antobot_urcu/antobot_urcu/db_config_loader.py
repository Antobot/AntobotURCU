import asyncpg
import asyncio
import json
import threading
import atexit
import yaml
import os
import traceback
from typing import Optional
import concurrent.futures
from ament_index_python.packages import get_package_share_directory

# Module-level singletons
_bg_loop: Optional[asyncio.AbstractEventLoop] = None
_bg_thread: Optional[threading.Thread] = None
_pool: Optional[asyncpg.pool.Pool] = None
_db_timeout: Optional[float] = None


def _bg_loop_thread_entry(loop_ready_event: threading.Event):
    global _bg_loop
    loop = asyncio.new_event_loop()
    _bg_loop = loop
    asyncio.set_event_loop(loop)
    loop_ready_event.set()
    loop.run_forever()


def _ensure_bg_loop():
    global _bg_thread, _bg_loop
    if _bg_loop is None:
        ready = threading.Event()
        _bg_thread = threading.Thread(target=_bg_loop_thread_entry, args=(ready,), daemon=True)
        _bg_thread.start()
        ready.wait()


async def _ensure_pool():
    global _pool, _db_timeout
    if _pool is None:
        try:
            parent_folder = get_package_share_directory('antobot_com_postgresql')
            yaml_file_path = os.path.join(parent_folder, "config/robot_config.yaml")
            print(f"[DB] Reading config from: {yaml_file_path}")
            with open(yaml_file_path, 'r') as file:
                config = yaml.safe_load(file)
            print(f"[DB] Config loaded: host={config.get('host')}, port={config.get('port')}, database={config.get('database')}, user={config.get('userName')}")
            _db_timeout = config.get('db_config_timeout')
            _pool = await asyncio.wait_for(
                asyncpg.create_pool(
                    host=config["host"],
                    port=int(config["port"]),
                    database=config["database"],
                    user=config["userName"],
                    password=config["passWord"],
                    min_size=1,
                    max_size=5,
                ),
                timeout=_db_timeout
            )
            print("[DB] Connection pool created successfully")
        except FileNotFoundError as e:
            print(f"[DB] ERROR: Config file not found - {e}")
            print(f"[DB] Full traceback:\n{traceback.format_exc()}")
            raise
        except yaml.YAMLError as e:
            print(f"[DB] ERROR: Failed to parse YAML config - {e}")
            print(f"[DB] Full traceback:\n{traceback.format_exc()}")
            raise
        except asyncpg.PostgresError as e:
            print(f"[DB] ERROR: Failed to connect to PostgreSQL - {e}")
            print(f"[DB] Full traceback:\n{traceback.format_exc()}")
            raise
        except Exception as e:
            print(f"[DB] ERROR: Unexpected error during pool creation - {e}")
            print(f"[DB] Full traceback:\n{traceback.format_exc()}")
            raise
    return _pool


async def load_config_from_db(config_name: str):
    pool = await _ensure_pool()

    async with pool.acquire() as con:
        try:
            result = await con.fetchrow(
                "SELECT content FROM cfg_robot WHERE name=$1 AND is_active=true",
                config_name,
            )
        except asyncpg.PostgresError as e:
            print(f"[DB] ERROR: Query failed - {e}")
            print(f"[DB] Full traceback:\n{traceback.format_exc()}")
            return {}
        except Exception as e:
            print(f"[DB] ERROR: Unexpected error during query - {e}")
            print(f"[DB] Full traceback:\n{traceback.format_exc()}")
            return {}

    if result:
        return json.loads(result["content"])
    else:
        return {}


def _run_coroutine_sync(coro, timeout: Optional[float] = None):
    _ensure_bg_loop()
    # Schedule the coroutine on the background loop and wait for result
    fut = asyncio.run_coroutine_threadsafe(coro, _bg_loop)
    try:
        if timeout is None:
            return fut.result()    
        else:
            return fut.result(timeout=timeout)
    except concurrent.futures.TimeoutError:
        try:
            fut.cancel()
        except Exception:
            pass
        raise


def _read_from_cfg_robot_db(config_name: str, timeout: Optional[float] = None):
    print(f"[DB] Loading config: {config_name}")
    try:
        config = _run_coroutine_sync(load_config_from_db(config_name), timeout=timeout)
    except concurrent.futures.TimeoutError:
        print(f"[DB] Failed to load config '{config_name}': Timeout after {timeout} seconds")
        print(f"[DB] Full traceback:\n{traceback.format_exc()}")
        return {}
    except Exception as e:
        print(f"[DB] Failed to load config '{config_name}': {type(e).__name__}: {e}")
        print(f"[DB] Full traceback:\n{traceback.format_exc()}")
        return {}

    if config:
        print(f"[DB] Config '{config_name}' loaded successfully. Keys: {list(config.keys())}")
    else:
        print(f"[DB] Config '{config_name}' NOT found or empty!")

    return config


def _read_from_file(config_path: str):
    if not config_path:
        return {}
    try:
        with open(config_path, 'r') as yamlfile:
            data = yaml.safe_load(yamlfile)
            if data is None:
                return {}
            return data
    except Exception as e:
        print(f"[File] Failed to read config from {config_path}: {e}")
        return {}


def get_robot_config(config_name: str, config_path: Optional[str] = None):
    """
    New config retrieval entry point:
    1) Prefer reading from cfg_robot (database) with a timeout (see `db_timeout` in config file);
    2) If the DB yields no result or times out, try reading YAML from the provided `config_path`;
    Returns a dict (empty dict if not found).
    """
    # 1. Try reading from cfg_robot (database)
    database_cfg = False
    if database_cfg:
        cfg = _read_from_cfg_robot_db(config_name, timeout=_db_timeout)
        if cfg:
            return cfg

    # 2. Fallback to filesystem
    if config_path:
        file_cfg = _read_from_file(config_path)
        if file_cfg:
            print(f"[File] Loaded config for '{config_name}' from provided path: {config_path}")
            return file_cfg

    # All attempts failed, return empty dict
    print(f"[get_robot_config] Config '{config_name}' not found in DB or files.")
    return {}


async def _close_pool():
    global _pool
    if _pool is not None:
        try:
            await _pool.close()
        except Exception:
            pass
        _pool = None


def _shutdown_bg_loop():
    global _bg_loop
    if _bg_loop is not None:
        try:
            asyncio.run_coroutine_threadsafe(_close_pool(), _bg_loop).result()
            _bg_loop.call_soon_threadsafe(_bg_loop.stop)
        except Exception:
            pass


atexit.register(_shutdown_bg_loop)

