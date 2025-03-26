import Image from "next/image";
import React from "react";

const FriendItem = ({ userInfo }) => {
  return (
    <div className="w-full h-14 px-4 rounded-2xl bg-primaryLightTheme flex items-center justify-between">
      <div className="flex items-center gap-2">
        <Image
          src={"/avatar.png"}
          width={50}
          height={50}
          className="size-8 rounded-xl object-cover -mr-0.5"
          alt={`کاربر ${userInfo.nickName}`}
        />

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
