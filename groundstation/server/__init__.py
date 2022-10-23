import os

from flask import Flask
from flask_socketio import SocketIO

socketio = SocketIO(async_mode='threading')


def create_app(script_info=None):
    # instantiate the app
    app = Flask(
        __name__,
        template_folder="../client/templates",
        static_folder="../client/static",
    )

    # set config
    app_settings = os.getenv("APP_SETTINGS")
    app.config.from_object(app_settings)

    # register blueprints
    from .main.views import main_blueprint

    app.register_blueprint(main_blueprint)

    socketio.init_app(app, cors_allowed_origins="*")

    # shell context for flask cli
    @app.shell_context_processor
    def shell():
        return {
            "app": app,
        }

    return app
