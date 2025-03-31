const express = require('express');
const next = require('next');
const http = require('http');
const { rpsSocket } = require('./src/sockets/rpsSocket');

const dev = process.env.NODE_ENV !== 'production';
const app = next({ dev });
const handle = app.getRequestHandler();
const port = process.env.PORT || 3000;

app.prepare().then(() => {
  const server = express();
  const httpServer = http.createServer(server);

  rpsSocket(httpServer); // اتصال WebSocket

  server.all('*', (req, res) => {
    return handle(req, res);
  });

  httpServer.listen(port, (err) => {
    if (err) throw err;
    console.log(`🚀 server running in port ${port}`);
  });
});
