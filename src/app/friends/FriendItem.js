"use client";

import { SocketContextData } from "@/context/SocketContext";
import Image from "next/image";
import React, { useContext } from "react";

const FriendItem = ({ userInfo }) => {
  const { onlineUsers } = useContext(SocketContextData);

  return (
    <div className="w-full h-14 px-4 rounded-2xl bg-primaryLightTheme flex items-center justify-between">
      <div className="flex items-center gap-2">
        <div className="relative">
          {onlineUsers.some((item) => item.userId === userInfo._id) ? (
            <div className="absolute -bottom-[1px] -left-[1px] flex size-2.5">
              <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-success opacity-75"></span>
              <span className="relative flex items-center justify-center rounded-full size-2.5 bg-success text-white font-bold text-xs"></span>
            </div>
          ) : (
            <div className="size-2.5 rounded-full bg-danger absolute -bottom-[1px] -left-[1px]"></div>
          )}

          <Image
            src={"/avatar.png"}
            width={50}
            height={50}
            className="size-8 rounded-xl object-cover -mr-0.5"
            alt={`کاربر ${userInfo.nickName}`}
          />
        </div>

        <div className="flex flex-col">
          <span className="text-xs">{userInfo.nickName}</span>
          <span className="text-[10px] text-blueColor">
            {userInfo.userName}
          </span>
        </div>
      </div>
    </div>
  );
};

export default FriendItem;
