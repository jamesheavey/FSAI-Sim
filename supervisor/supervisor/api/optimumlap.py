from .. import app, supervisor_state
from flask import abort, request


@app.route("/optimumlap", methods=["POST"])
def upload_optimumlap_file():
    if "file" not in request.files:
        abort(400)
    file = request.files["file"]
    if file.filename == "":
        abort(400)
    supervisor_state.options.optimumlap_files.upload_file(file.filename, file)
    return ""


@app.route("/optimumlap/<filename>", methods=["DELETE"])
def delete_optimumlap_track(filename):
    res = supervisor_state.options.optimumlap_files.delete_file(filename)
    if not res:
        abort(400)
    return ""
