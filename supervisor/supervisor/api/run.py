from .. import app, api_runner
from flask import abort, jsonify, request


@app.route("/options", methods=["GET"])
def all_options():
    return jsonify(api_runner.get_options())


@app.route("/run", methods=["GET"])
def get_config():
    return jsonify(api_runner.get_configuration())


@app.route("/run", methods=["POST"])
def run():
    if not request.is_json:
        abort(400)
    data = request.json

    try:
        resp = api_runner.run(data)
    except Exception as e:
        return jsonify({"error": str(e)}), 400

    return jsonify(resp)


@app.route("/run", methods=["DELETE"])
def stop():
    try:
        return jsonify(api_runner.stop())
    except Exception as e:
        return jsonify({"error": str(e)}), 400
