const express = require("express");
const http = require("http");
const { Server } = require("socket.io");

const app = express();
const server = http.createServer(app);
const io = new Server(server, {
  cors: { origin: "*" } // Allow connections from any origin
});

io.on("connection", (socket) => {
  console.log(`New client connected: ${socket.id}`);

  socket.on("offer", (offer) => {
    console.log("Offer received");
    socket.broadcast.emit("offer", offer);
  });

  socket.on("answer", (answer) => {
    console.log("Answer received");
    socket.broadcast.emit("answer", answer);
  });

  socket.on("candidate", (candidate) => {
    console.log("ICE Candidate received");
    socket.broadcast.emit("candidate", candidate);
  });

  socket.on("disconnect", () => {
    console.log(`Client disconnected: ${socket.id}`);
  });
});

const PORT = 3000;  // Change this if needed
server.listen(PORT, () => console.log(`Signaling server running on port ${PORT}`));
