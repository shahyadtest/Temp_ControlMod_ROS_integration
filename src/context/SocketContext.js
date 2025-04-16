"use client";
import { siteURL } from "@/services/API";
import { useUser } from "@/store/useUser";
import React, { createContext, useEffect, useState } from "react";
import { io } from "socket.io-client";

export const SocketContextData = createContext();

const socket = io(siteURL);

const SocketContext = ({ children, userInfo }) => {
  const user = userInfo?.user;
  const [onlineUsers, setOnlineUsers] = useState([]);

  useEffect(() => {
    if (!user) return;

    socket.emit("userInfo", {
      userId: user._id,
      userName: user.userName,
      nickName: user.nickName,
    });

    socket.on("onlineUsers", (users) => setOnlineUsers(users));
  }, []);

  return (
    <SocketContextData.Provider value={{ onlineUsers }}>
      {children}
    </SocketContextData.Provider>
  );
};

export default SocketContext;
