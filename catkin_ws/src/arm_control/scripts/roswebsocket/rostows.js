const { spawn } = require('child_process');
const http = require('http');
const socketIo = require('socket.io');
const yaml = require('js-yaml');

// Create HTTP server and attach Socket.IO
const server = http.createServer();
const io = socketIo(server, {
  cors: {
    origin: "*",  // Allow connections from all origins
    methods: ["GET", "POST"]
  }
});

// Start Socket.IO server on port 8080
server.listen(8080, () => {
  console.log('Socket.IO server running at http://localhost:8080');
});

// Broadcast function to send data to all connected clients
function broadcast(data) {
  io.emit('tf', data);  // Emit the 'tf' event to all connected clients
}

// Spawn the rostopic echo command to listen to /tf
const rosTopic = spawn('rostopic', ['echo', '/tf', '--noarr']);

// Buffer to store multi-line messages
let messageBuffer = '';

rosTopic.stdout.on('data', (data) => {
  // Convert the incoming Buffer to a UTF-8 string
  const decodedData = data.toString('utf8');
  messageBuffer += decodedData;

  // Log the raw data to see what is being received
  console.log('Raw data from rostopic echo:', decodedData);

  // Process each line of the decoded message
  const lines = messageBuffer.split('\n');
  let completeMessage = '';

  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];
    completeMessage += line + '\n';

    // If the message is complete (no indentation for the next line), we broadcast it
    if ((i + 1 < lines.length && !lines[i + 1].startsWith(' ')) || i + 1 === lines.length) {
      // Log the complete message to analyze the structure
      console.log("Complete message received:", completeMessage);

      // Check if the message contains metadata about the transforms array
      if (completeMessage.includes('<array type: geometry_msgs/TransformStamped')) {
        console.log('Received metadata, skipping transformation data.');
      } else {
        try {
          // Parse the message as YAML
          const parsedMessage = yaml.load(completeMessage.trim());

          // Check if parsedMessage is valid
          if (parsedMessage && parsedMessage.transforms && Array.isArray(parsedMessage.transforms)) {
            // If transforms array exists, broadcast the data
            console.log('Parsed transforms data:', parsedMessage.transforms);
            broadcast({ event: 'tf', data: parsedMessage });
          } else {
            console.error('Parsed message missing transforms array:', parsedMessage);
          }
        } catch (e) {
          console.error('Error parsing the data:', e);
        }
      }

      completeMessage = '';  // Reset for the next message
    }
  }

  // Clear the buffer after processing
  messageBuffer = '';
});

