import logging

def setup_logger(name: str, level=logging.INFO):
    logger = logging.getLogger(name)
    logger.setLevel(level)
    handler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    
    # Évite de dupliquer les handlers si le logger existe déjà
    if not logger.handlers:
        logger.addHandler(handler)
        
    return logger

logger = setup_logger("robot-traceur")
