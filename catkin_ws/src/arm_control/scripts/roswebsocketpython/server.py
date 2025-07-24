import subprocess
import socketio
import yaml

# Initialize Socket.IO client
sio = socketio.Server(cors_allowed_origins="*")
app = socketio.WSGIApp(sio)

# Broadcast function to send data to all connected clients
def broadcast(data):
    sio.emit('tf', data)  # Emit the 'tf' event to all connected clients

# Start the Socket.IO server (you'll need a WSGI server like 'eventlet' or 'gevent' to run the app)
if __name__ == '__main__':
    from wsgiref.simple_server import make_server
    server = make_server('localhost', 8080, app)
    print('Socket.IO server running at http://localhost:8080')
    server.serve_forever()

# Spawn the rostopic echo command to listen to /tf
ros_topic = subprocess.Popen(['rostopic', 'echo', '/tf', '--noarr'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

# Buffer to store multi-line messages
message_buffer = ''

while True:
    # Read the data from stdout (this would be an infinite loop, so add your own break logic if needed)
    data = ros_topic.stdout.readline()
    if not data:
        break

    # Convert the incoming byte data to a UTF-8 string
    decoded_data = data.decode('utf-8')
    message_buffer += decoded_data

    # Log the raw data to see what is being received
    print('Raw data from rostopic echo:', decoded_data)

    # Process each line of the decoded message
    lines = message_buffer.split('\n')
    complete_message = ''

    for i, line in enumerate(lines):
        complete_message += line + '\n'

        # If the message is complete (no indentation for the next line), we broadcast it
        if (i + 1 < len(lines) and not lines[i + 1].startswith(' ')) or i + 1 == len(lines):
            # Log the complete message to analyze the structure
            print("Complete message received:", complete_message)

            # Check if the message contains metadata about the transforms array
            if '<array type: geometry_msgs/TransformStamped' in complete_message:
                print('Received metadata, skipping transformation data.')
            else:
                try:
                    # Parse the message as YAML
                    parsed_message = yaml.safe_load(complete_message.strip())

                    # Check if parsed_message is valid
                    if parsed_message and 'transforms' in parsed_message and isinstance(parsed_message['transforms'], list):
                        # If transforms array exists, broadcast the data
                        print('Parsed transforms data:', parsed_message['transforms'])
                        broadcast({'event': 'tf', 'data': parsed_message})
                    else:
                        print('Parsed message missing transforms array:', parsed_message)
                except Exception as e:
                    print('Error parsing the data:', e)

            complete_message = ''  # Reset for the next message

    # Clear the buffer after processing
    message_buffer = ''

