const { Server } = require("socket.io");
const axios = require("axios");

let io;
const onlineUsers = {}; // online users
let waitingPlayer = null;
const gameMoves = {};

const rpsSocket = (httpServer) => {
  if (!io) {
    io = new Server(httpServer, { cors: { origin: "*" } });

    io.on("connection", (socket) => {
      console.log(`🔵 User connected: ${socket.id}`);

      socket.on("userInfo", async ({ userId, userName, nickName }) => {
        onlineUsers[socket.id] = { userId, userName, nickName };
        io.emit("onlineUsers", Object.values(onlineUsers));
      });

      socket.on("findGame", () => handleFindGame(socket));

      socket.on("makeMove", ({ roomId, move }) =>
        handleMakeMove(socket, roomId, move)
      );

      socket.on("disconnect", () => handleDisconnect(socket));
    });
  }
};

// find opponent & create a room
const handleFindGame = async (socket) => {
  if (waitingPlayer) {
    const roomId = `room-${onlineUsers[waitingPlayer].userId}-${
      onlineUsers[socket.id].userId
    }`;

    try {
      const createRoomRes = await fetch(
        `http://localhost:3000/api/rps/create-room`,
        {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            roomId,
            player1: onlineUsers[waitingPlayer].userId,
            player2: onlineUsers[socket.id].userId,
          }),
        }
      );

      if (createRoomRes.ok) {
        io.to(waitingPlayer).emit("gameFound", {
          roomId,
          opponent: onlineUsers[socket.id],
        });
        io.to(socket.id).emit("gameFound", {
          roomId,
          opponent: onlineUsers[waitingPlayer],
        });

        // save moves
        gameMoves[roomId] = {};
      }

      waitingPlayer = null;
    } catch (error) {
      console.error("❌ Error creating game room:", error);
    }
  } else {
    waitingPlayer = socket.id;
    socket.emit("waiting");
  }
};

// save moves(analyse) & game result
const handleMakeMove = async (socket, roomId, move) => {
  if (!gameMoves[roomId]) return;

  gameMoves[roomId][onlineUsers[socket.id]] = move; // save move

  const players = Object.keys(gameMoves[roomId]);
  if (players.length === 2) {
    // both players make their moves show result
    const { player1, player2 } = gameMoves[roomId];
    const result = determineWinner(player1, player2);

    const winner =
      result === "draw"
        ? "draw"
        : result === "player1"
        ? Object.keys(gameMoves[roomId])[0]
        : Object.keys(gameMoves[roomId])[1];

    // save moves & result in db
    try {
      await axios.post("http://localhost:3000/api/game/finish", {
        roomId,
        moves: gameMoves[roomId],
        winner,
      });

      io.to(roomId).emit("gameOver", { result, winner });
      delete gameMoves[roomId]; // clear moves
    } catch (error) {
      console.error("❌ Error finishing game:", error);
    }
  }
};

// user disconnect handler
const handleDisconnect = (socket) => {
  console.log(`🔴 User disconnected: ${socket.id}`);
  delete onlineUsers[socket.id];
  if (waitingPlayer === socket.id) waitingPlayer = null;
  io.emit("onlineUsers", Object.values(onlineUsers));
};

// match result function
const determineWinner = (move1, move2) => {
  const rules = { rock: "scissors", paper: "rock", scissors: "paper" };
  if (move1 === move2) return "draw";
  return rules[move1] === move2 ? "player1" : "player2";
};

module.exports = { rpsSocket };
