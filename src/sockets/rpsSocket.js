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

// New: Store pending invitations
const pendingInvitations = {};

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

        // Send any pending invitations to the user
        if (pendingInvitations[userId]) {
          pendingInvitations[userId].forEach((invitation) => {
            socket.emit("gameInvitation", {
              from: invitation.from,
              invitationId: invitation.invitationId,
            });
          });
        }
      });

      socket.on("findGame", () => handleFindGame(socket)); // حفظ findGame
      socket.on("makeMove", ({ roomId, move }) =>
        handleMakeMove(socket, roomId, move)
      );
      socket.on("joinRoom", (roomId) => socket.join(roomId));
      socket.on("cancelGame", () => handleDisconnect(socket));

      // New: Handle game invitations
      socket.on("inviteFriend", ({ friendId }) =>
        handleInviteFriend(socket, friendId)
      );
      socket.on("acceptInvitation", ({ invitationId }) =>
        handleAcceptInvitation(socket, invitationId)
      );
      socket.on("rejectInvitation", ({ invitationId }) =>
        handleRejectInvitation(socket, invitationId)
      );
    });
  }
};

// New: Handle friend invitation
const handleInviteFriend = (socket, friendId) => {
  const inviterId = socket.userId;

  // Check if friend is online
  if (!onlineUsers[friendId]) {
    socket.emit("invitationError", { message: "دوست شما آنلاین نیست!" });
    return;
  }

  // Create invitation ID
  const invitationId = `inv-${inviterId}-${friendId}-${Date.now()}`;

  // Store invitation
  if (!pendingInvitations[friendId]) {
    pendingInvitations[friendId] = [];
  }

  pendingInvitations[friendId].push({
    invitationId,
    from: {
      userId: inviterId,
      userName: onlineUsers[inviterId].userName,
      nickName: onlineUsers[inviterId].nickName,
    },
    timestamp: Date.now(),
  });

  // Send invitation to friend
  io.to(onlineUsers[friendId].socketId).emit("gameInvitation", {
    invitationId,
    from: {
      userId: inviterId,
      userName: onlineUsers[inviterId].userName,
      nickName: onlineUsers[inviterId].nickName,
    },
  });

  // Notify inviter that invitation was sent
  socket.emit("invitationSent", {
    to: friendId,
    invitationId,
  });

  // Set timeout to automatically remove invitation after 2 minutes
  setTimeout(() => {
    if (pendingInvitations[friendId]) {
      pendingInvitations[friendId] = pendingInvitations[friendId].filter(
        (inv) => inv.invitationId !== invitationId
      );

      if (pendingInvitations[friendId].length === 0) {
        delete pendingInvitations[friendId];
      }

      // Notify both users that invitation expired
      if (onlineUsers[inviterId]) {
        io.to(onlineUsers[inviterId].socketId).emit("invitationExpired", {
          invitationId,
        });
      }

      if (onlineUsers[friendId]) {
        io.to(onlineUsers[friendId].socketId).emit("invitationExpired", {
          invitationId,
        });
      }
    }
  }, 120000); // 2 minutes
};

// New: Handle invitation acceptance
const handleAcceptInvitation = async (socket, invitationId) => {
  const accepterId = socket.userId;

  // Find the invitation
  let invitation = null;
  let inviterUserId = null;

  if (pendingInvitations[accepterId]) {
    invitation = pendingInvitations[accepterId].find(
      (inv) => inv.invitationId === invitationId
    );
    if (invitation) {
      inviterUserId = invitation.from.userId;
    }
  }

  if (!invitation) {
    socket.emit("invitationError", {
      message: "دعوت نامه معتبر نیست یا منقضی شده است!",
    });
    return;
  }

  // Check if inviter is still online
  if (!onlineUsers[inviterUserId]) {
    socket.emit("invitationError", { message: "دعوت کننده آنلاین نیست!" });

    // Remove invitation
    pendingInvitations[accepterId] = pendingInvitations[accepterId].filter(
      (inv) => inv.invitationId !== invitationId
    );

    if (pendingInvitations[accepterId].length === 0) {
      delete pendingInvitations[accepterId];
    }

    return;
  }

  // Create a room for the game
  const roomId = `room-${inviterUserId}-${accepterId}-${Date.now()}`;

  try {
    const createRoomRes = await fetch(`${baseURL}/api/rps/create-room`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        roomId,
        player1: inviterUserId,
        player2: accepterId,
        isInvitation: true,
      }),
    });

    if (createRoomRes.ok) {
      // Join both sockets to the room
      const inviterSocket = io.sockets.sockets.get(
        onlineUsers[inviterUserId].socketId
      );
      inviterSocket.join(roomId);
      socket.join(roomId);

      // Notify both players that game is starting
      io.to(onlineUsers[inviterUserId].socketId).emit("gameFound", {
        roomId,
        opponent: onlineUsers[accepterId],
        playerTurn: onlineUsers[inviterUserId],
        isInvitedGame: true,
      });

      io.to(socket.id).emit("gameFound", {
        roomId,
        opponent: onlineUsers[inviterUserId],
        playerTurn: onlineUsers[inviterUserId],
        isInvitedGame: true,
      });

      gameMoves[roomId] = {};
      playerTurn[roomId] = inviterUserId; // نوبت بازیکن اول (دعوت کننده)

      // Remove invitation
      pendingInvitations[accepterId] = pendingInvitations[accepterId].filter(
        (inv) => inv.invitationId !== invitationId
      );

      if (pendingInvitations[accepterId].length === 0) {
        delete pendingInvitations[accepterId];
      }
    }
  } catch (error) {
    console.error("❌ Error creating invited game room:", error);
    socket.emit("invitationError", { message: "خطا در ایجاد اتاق بازی!" });
  }
};

// New: Handle invitation rejection
const handleRejectInvitation = (socket, invitationId) => {
  const rejecterId = socket.userId;

  // Find the invitation
  let invitation = null;
  let inviterUserId = null;

  if (pendingInvitations[rejecterId]) {
    invitation = pendingInvitations[rejecterId].find(
      (inv) => inv.invitationId === invitationId
    );
    if (invitation) {
      inviterUserId = invitation.from.userId;
    }
  }

  if (!invitation) {
    return; // Invitation doesn't exist or already expired
  }

  // Remove invitation
  pendingInvitations[rejecterId] = pendingInvitations[rejecterId].filter(
    (inv) => inv.invitationId !== invitationId
  );

  if (pendingInvitations[rejecterId].length === 0) {
    delete pendingInvitations[rejecterId];
  }

  // Notify inviter that invitation was rejected
  if (onlineUsers[inviterUserId]) {
    io.to(onlineUsers[inviterUserId].socketId).emit("invitationRejected", {
      invitationId,
      by: {
        userId: rejecterId,
        userName: onlineUsers[rejecterId]?.userName || "کاربر",
        nickName: onlineUsers[rejecterId]?.nickName,
      },
    });
  }

  // Confirm to rejecter
  socket.emit("invitationRejected", { invitationId, status: "success" });
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
