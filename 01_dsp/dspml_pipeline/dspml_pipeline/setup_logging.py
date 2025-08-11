# Internal logging wrapper

import logging
import sys

def setup_logging(verbose: bool = True) -> None:
    level = logging.DEBUG if verbose else logging.WARNING

    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        stream=sys.stdout
    )

    logging.info("Logging enabled.")