from . import ros, socketio, app
import atexit


def main(args=None):

    ros.init(args)

    ros.start()

    def cleanup():
        print("Cleanup triggered")
        ros.shutdown()
        ros.join()

    atexit.register(cleanup)
    socketio.run(app, host="0.0.0.0")


if __name__ == '__main__':
    main()
