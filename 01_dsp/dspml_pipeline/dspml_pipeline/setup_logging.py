# Internal logging wrapper

import logging
import sys

def setup_logging(verbose: bool = True) -> None:
    """
    Sets up logging in a standardized way for each file. Place the following at
    the top of each file where the logger is used:
        import logging
        logger = logging.getLogger(__name__)

    Args:
        verbose (bool): Enables or disables INFO logging.
    """
    
    level = logging.INFO if verbose else logging.WARNING

    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        stream=sys.stdout
    )

    logging.info("Logging enabled.")