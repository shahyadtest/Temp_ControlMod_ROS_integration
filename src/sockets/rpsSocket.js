const { Server } = require("socket.io");

let io;
const onlineUsers = {}; // online users
let waitingPlayer = null;
const gameMoves = {};
const playerTurn = {}; // نگهداری نوبت هر بازیکن
const baseURL = "http://localhost:3000";
// process.env.NODE_ENV === "development"
//   ? "http://localhost:3000"
//   : "https://chess-production-9ba7.up.railway.app";

const rpsSocket = (httpServer) => {
  if (!io) {
    io = new Server(httpServer, { cors: { origin: "*" } });

    io.on("connection", (socket) => {
      console.log(`🔵 User connected: ${socket.id}`);

      socket.on("userInfo", async ({ userId, userName, nickName }) => {
        socket.userId = userId; // userId رو توی سوکت ذخیره کن
        console.log(
          `Set socket.userId to ${socket.userId} for socket ${socket.id}`
        );
        onlineUsers[userId] = {
          socketId: socket.id,
          userName,
          nickName,
          userId,
        };
        io.emit("onlineUsers", Object.values(onlineUsers));
      });

      socket.on("findGame", () => handleFindGame(socket)); // حفظ findGame
      socket.on("makeMove", ({ roomId, move }) =>
        handleMakeMove(socket, roomId, move)
      );
      socket.on("joinRoom", (roomId) => socket.join(roomId));
      socket.on("cancelGame", () => handleDisconnect(socket));
    });
  }
};

// find opponent & create a room
const handleFindGame = async (socket) => {
  if (waitingPlayer) {
    console.log(waitingPlayer);
    const roomId = `room-${waitingPlayer.userId}-${
      socket.userId
    }-${Date.now()}`;

    try {
      const createRoomRes = await fetch(`${baseURL}/api/rps/create-room`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          roomId,
          player1: waitingPlayer.userId,
          player2: socket.userId,
        }),
      });

      if (createRoomRes.ok) {
        // جوین کردن سوکت‌ها به اتاق
        waitingPlayer.join(roomId);
        socket.join(roomId);

        io.to(waitingPlayer.id).emit("gameFound", {
          roomId,
          opponent: onlineUsers[socket.userId],
          playerTurn: onlineUsers[waitingPlayer.userId],
        });
        io.to(socket.id).emit("gameFound", {
          roomId,
          opponent: onlineUsers[socket.userId],
          playerTurn: onlineUsers[waitingPlayer.userId],
        });

        gameMoves[roomId] = {};
        playerTurn[roomId] = waitingPlayer.userId; // نوبت بازیکن اول
      }

      waitingPlayer = null;
    } catch (error) {
      console.error("❌ Error creating game room:");
    }
  } else {
    console.log("waitingPlayer");
    waitingPlayer = socket;
    socket.emit("waiting");
  }
};

// save moves(analyse) & game result
const handleMakeMove = async (socket, roomId, move) => {
  console.log(`Received move from ${socket.userId} in room ${roomId}: ${move}`);

  if (!gameMoves[roomId]) {
    console.log(`Room ${roomId} not found in gameMoves`);
    return;
  }

  const currentPlayer = socket.userId;
  console.log(`Current player: ${currentPlayer}`);

  // ذخیره حرکت بازیکن فعلی
  gameMoves[roomId][currentPlayer] = move;
  console.log(`Updated gameMoves[${roomId}]:`, gameMoves[roomId]);

  // چک کردن اینکه هر دو بازیکن حرکت کردن یا نه
  const playerKeys = Object.keys(gameMoves[roomId]);
  console.log(`Players who moved: ${playerKeys}`);

  if (
    playerKeys.length === 2 &&
    gameMoves[roomId][playerKeys[0]] &&
    gameMoves[roomId][playerKeys[1]]
  ) {
    const [player1, player2] = playerKeys;
    const move1 = gameMoves[roomId][player1];
    const move2 = gameMoves[roomId][player2];
    console.log(`Moves - ${player1}: ${move1}, ${player2}: ${move2}`);

    const result = determineWinner(move1, move2);
    const winner =
      result === "draw" ? "draw" : result === "player1" ? player1 : player2;
    console.log(`Game result: ${result}, Winner: ${winner}`);

    io.to(roomId).emit("gameOver", {
      result,
      winner,
      gameMoves: gameMoves[roomId],
    });
    console.log(`Sent gameOver to room ${roomId}`);
    gameMoves[roomId] = {};

    io.to(roomId).emit("waitingForOpponent", {
      message: "منتظر حرکت حریف باشید!",
      currentPlayer,
    });
  } else {
    io.to(roomId).emit("waitingForOpponent", {
      message: "منتظر حرکت حریف باشید!",
      currentPlayer,
    });
    console.log(`Sent waitingForOpponent to ${socket.userId}`);
  }
};

// user disconnect handler
const handleDisconnect = (socket) => {
  console.log(`🔴 User disconnected: ${socket.id}`);
  if (socket.userId) {
    delete onlineUsers[socket.userId];
  } else {
    // حذف با socketId در صورتی که userId نداشته باشه
    for (const key in onlineUsers) {
      if (onlineUsers[key].socketId === socket.id) {
        delete onlineUsers[key];
      }
    }
  }

  if (waitingPlayer && waitingPlayer.userId === socket.userId) {
    waitingPlayer = null;
  }
  io.emit("onlineUsers", Object.values(onlineUsers));
};

// match result function
const determineWinner = (move1, move2) => {
  const rules = { rock: "scissors", paper: "rock", scissors: "paper" };
  if (move1 === move2) return "draw";
  return rules[move1] === move2 ? "player1" : "player2";
};

module.exports = { rpsSocket };
