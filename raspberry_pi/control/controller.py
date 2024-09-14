import os
from flask import Flask, request, jsonify
from motor import MotorGpio, MotorController

app = Flask(__name__)

gpio_config = MotorGpio(
    GPIO_ENABLE_A = 1    # Left
    GPIO_ENABLE_B = 1    # Right
    GPIO_IN1 = 1         # Left - F
    GPIO_IN2 = 1         # Left - B
    GPIO_IN3 = 1         # Right - F
    GPIO_IN4 = 1         # Right - B
    
    PWM_HZ = 1000
)
motor_controller = MotorController(gpio_config)

@app.route('/')
def index():
    return send_file(os.path.join(os.path.dirname(os.path.realpath(__file__)), "controller.html"), mimetype="text/html")

@app.route('/move', methods=['POST'])
def move():
    direction = request.json.get('direction')
    speed = request.json.get('speed', 50)

    try:
        if direction == 'forward':
            motor_controller.move_forward(speed)
        elif direction == 'backward':
            motor_controller.move_backward(speed)
        elif direction == 'left':
            motor_controller.turn_left(speed)
        elif direction == 'right':
            motor_controller.turn_right(speed)
        elif direction == 'stop':
            motor_controller.stop()
        else:
            return jsonify({'status': 'error', 'message': 'Invalid direction'}), 400

        return jsonify({'status': 'success', 'message': f'Moving {direction} at speed {speed}'})

    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/stop', methods=['POST'])
def stop():
    motor_controller.stop()
    return jsonify({'status': 'success', 'message': 'Stopped motor'})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)
    motor_controller.cleanup()
