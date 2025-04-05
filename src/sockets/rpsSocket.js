const { Server } = require("socket.io");
const axios = require("axios");

let io;
const onlineUsers = {}; // online users
let waitingPlayer = null;
const gameMoves = {};
const playerTurn = {}; // نگهداری نوبت هر بازیکن

const rpsSocket = (httpServer) => {
  if (!io) {
    io = new Server(httpServer, { cors: { origin: "*" } });

    io.on("connection", (socket) => {
      console.log(`🔵 User connected: ${socket.id}`);

      socket.on("userInfo", async ({ userId, userName, nickName }) => {
        onlineUsers[socket.id] = { userId, userName, nickName };
        io.emit("onlineUsers", Object.values(onlineUsers));
      });

      socket.on("findGame", () => handleFindGame(socket)); // حفظ findGame

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
    }-${Date.now()}`;

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
          playerTurn: onlineUsers[waitingPlayer],
        });
        io.to(socket.id).emit("gameFound", {
          roomId,
          opponent: onlineUsers[waitingPlayer],
          playerTurn: onlineUsers[waitingPlayer],
        });

        // save moves
        gameMoves[roomId] = {};
        playerTurn[roomId] = onlineUsers[waitingPlayer].userId; // first player starts
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

  const currentPlayer = onlineUsers[socket.id].userId;
  const currentTurn = playerTurn[roomId];

  // Check if it's the player's turn
  if (currentPlayer !== currentTurn) {
    socket.emit("notYourTurn", "It's not your turn yet!");
    return;
  }

  gameMoves[roomId][onlineUsers[socket.id]] = move; // save move

  // Switch turn to the other player
  playerTurn[roomId] =
    currentPlayer === onlineUsers[roomId].player1
      ? onlineUsers[roomId].player2
      : onlineUsers[roomId].player1;

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
    io.to(roomId).emit("gameOver", { result, winner, gameMoves });
    delete gameMoves[roomId]; // clear moves
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
