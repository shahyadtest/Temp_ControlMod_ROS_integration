"use client";
import { toFarsiNumber } from "@/helper/helper";
import React, { useState } from "react";
import FriendRequestItem from "./FriendRequestItem";

const FriendRequests = ({ data }) => {
  const [openDrawer, setOpenDrawer] = useState(false);

  return (
    <>
      <div className="flex items-center gap-1">
        <button
          onClick={() => setOpenDrawer(true)}
          className="bg-gradient-to-b from-white to-gray-400 bg-clip-text text-transparent"
        >
          درخواست های دوستی
        </button>

        <div className="size-5 text-xs bg-red-600 flex justify-center items-center rounded-lg">
          {toFarsiNumber(data.length)}
        </div>
      </div>

      <div
        onClick={() => setOpenDrawer(false)}
        className={`${
          openDrawer ? "opacity-100 visible" : "opacity-0 invisible"
        } bg-black bg-opacity-60 w-full h-full fixed top-0 bottom-0 right-0 z-40 transition-all duration-300`}
      ></div>

      <div
        className={`${
          openDrawer ? "bottom-0" : "-bottom-full"
        } w-full h-96 right-0 bg-primaryDarkTheme absolute z-50 rounded-t-3xl transition-all duration-300`}
      >
        <div className="w-full h-full p-5 relative flex flex-col gap-3 pt-12">
          <span className="w-20 h-2 rounded-full bg-grayColor absolute top-4 left-2/4 -translate-x-2/4 block"></span>

          <h2 className="bg-gradient-to-b from-white to-gray-400 bg-clip-text text-transparent">
            درخواست های در انتظار تایید
          </h2>

          {data?.length ? (
            data.map((item) => (
              <FriendRequestItem key={item._id} userInfo={item} />
            ))
          ) : (
            <div className="w-full flex justify-center items-center bg-secondaryDarkTheme h-14 rounded-2xl text-xs">
              <span>درخواستی وجود ندارد !</span>
            </div>
          )}
        </div>
      </div>
    </>
  );
};

export default FriendRequests;
