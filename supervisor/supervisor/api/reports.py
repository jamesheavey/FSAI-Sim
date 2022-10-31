from .. import app, supervisor_state
from flask import abort, jsonify, request, send_file
from json import JSONDecodeError
import io


@app.route("/reports", methods=["GET"])
def get_reports():
    return jsonify(supervisor_state.reports.to_dict())


@app.route("/reports", methods=["POST"])
def upload_report():
    if "file" not in request.files:
        abort(400)
    file = request.files["file"]
    if file.filename == "":
        abort(400)
    try:
        return jsonify(supervisor_state.reports.upload_file(file))
    except (FileExistsError, FileNotFoundError, JSONDecodeError):
        abort(400)


@app.route("/reports/<filename>", methods=["GET"])
def get_report(filename):
    try:
        return send_file(supervisor_state.reports.get_file(filename))
    except FileNotFoundError:
        abort(404)


@app.route("/reports/<filename>", methods=["DELETE"])
def delete_report(filename):
    try:
        supervisor_state.reports.delete_file(filename)
        return jsonify({}), 204
    except FileNotFoundError:
        abort(404)


@app.route("/reports/<filename>/zip", methods=["GET"])
def get_report_zip(filename):
    try:
        zip_path = supervisor_state.reports.get_zip(filename)
        return send_file(
            zip_path,
            as_attachment=True,
            attachment_filename=f"{filename}.zip",
            mimetype="application/zip")
    except FileNotFoundError:
        abort(404)


@app.route("/reports/<filename>/frame/<frame_key>/<frame_id>", methods=["GET"])
def get_frame(filename, frame_key, frame_id):
    try:
        frame = supervisor_state.reports.get_frame_jpeg(filename, frame_key, frame_id)
        return send_file(io.BytesIO(frame), mimetype="image/jpeg")
    except FileNotFoundError:
        abort(404)
