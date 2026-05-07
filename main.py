"""
main.py
=======
Main entry point for the Robot Traceur PDF system.
Redirects to the internal CLI application.
"""

from app.cli import run_cli

if __name__ == "__main__":
    run_cli()
