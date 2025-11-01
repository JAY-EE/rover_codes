// signaling-server.js
const WebSocket = require("ws");

const wss = new WebSocket.Server({ port: 8080 });

wss.on("connection", ws => {
  ws.on("message", message => {
    // Just broadcast to all clients (simple signaling)
    wss.clients.forEach(client => {
      if (client !== ws && client.readyState === WebSocket.OPEN) {
        client.send(message);
      }
    });
  });
});

console.log("Signaling server running on ws://localhost:8080");
