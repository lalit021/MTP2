const io = require('socket.io-client');

// Replace with the actual server address
const socket = io('http://localhost:8080');  // Change to your server's URL

socket.on('connect', () => {
  console.log('Connected to Socket.IO server');
  
  // Send a message to the server
  socket.emit('myEvent', { message: 'Hello from the client!' });
});

socket.on('tf', (data) => {
  console.log('Received from server:', data);
});

socket.on('disconnect', () => {
  console.log('Disconnected from server');
});

