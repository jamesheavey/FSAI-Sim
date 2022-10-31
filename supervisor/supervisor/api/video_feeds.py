from .. import app, supervisor_state
from flask import Response, abort, jsonify
import time


def stream(image_store, scale=1):
    """
    Constantly stream latest frame, as soon as it's updated
    """
    last_rendered_frame = -1
    while True:
        # Check if new frame is available
        if image_store.last_frame_id <= last_rendered_frame:
            time.sleep(0.1)
            continue
        last_rendered_frame = image_store.last_frame_id

        img = image_store.get_last_image_jpeg(scale)
        if img is None:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + img + b'\r\n')


@app.route("/video_feeds")
def get_video_feeds():
    """
    Return list of available video feeds
    """
    return jsonify(list(supervisor_state.image_stores.keys()))


@app.route("/video_feeds/<feed>")
def video_feed(feed):
    """
    Return selected video feed
    """
    # If feed is not an image store, throw a 404
    if feed not in supervisor_state.image_stores:
        abort(404)
    return Response(stream(supervisor_state.image_stores[feed]),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route("/video_feeds/<feed>/<float:scale>")
def scaled_video_feed(feed, scale):
    """
    Return selected scaled video feed
    """
    # If feed is not an image store, throw a 404
    if feed not in supervisor_state.image_stores:
        abort(404)
    scale = min(scale, 1)
    return Response(stream(supervisor_state.image_stores[feed], scale),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
