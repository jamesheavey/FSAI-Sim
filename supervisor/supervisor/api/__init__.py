from .. import app, supervisor_state
from flask import abort, send_file, jsonify

from .reports import *        # noqa: F401, F403
from .optimumlap import *     # noqa: F401, F403
from .video_feeds import *    # noqa: F401, F403
from .run import *        # noqa: F401, F403


@app.route("/")
def index():
    return app.send_static_file('index.html')


@app.route("/car_preview/<car_name>")
def get_car_preview(car_name):
    path = supervisor_state.options.get_car_preview(car_name)
    if path is None:
        abort(404)
    return send_file(path)


@app.route("/fingerprint")
def get_fingerprint():
    return jsonify({
        "id": "f21ai-sim"
    })
