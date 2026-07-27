from typing import Any, Optional

try:
    import torch.distributed as dist
except ImportError:
    dist = None


def _distributed_ready() -> bool:
    """Return whether the current PyTorch build has an initialized process group"""
    if dist is None:
        return False
    is_initialized = getattr(dist, "is_initialized", None)
    if not callable(is_initialized):
        return False
    try:
        return bool(is_initialized())
    except (RuntimeError, AttributeError):
        return False


def get_rank(group: Optional[Any] = None):
    if not _distributed_ready():
        return 0
    return dist.get_rank(group)


def get_world_size(group: Optional[Any] = None):
    if not _distributed_ready():
        return 1
    return dist.get_world_size(group)


def barrier(group: Optional[Any] = None):
    if _distributed_ready():
        dist.barrier(group)


class rank_gate:
    '''
    Execute the function on rank 0 first, followed by all other ranks. Useful when caches may need to be populated in a distributed environment.
    '''
    def __init__(self, func=None):
        self.func = func

    def __call__(self, *args, **kwargs):
        rank = get_rank()
        if rank == 0:
            result = self.func(*args, **kwargs)
        barrier()
        if rank > 0:
            result = self.func(*args, **kwargs)
        return result

    def __enter__(self, *args, **kwargs):
        if get_rank() > 0:
            barrier()

    def __exit__(self, *args, **kwargs):
        if get_rank() == 0:
            barrier()
