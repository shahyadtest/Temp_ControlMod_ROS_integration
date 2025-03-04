"use client";
import React, { useEffect } from "react";

const Auth = () => {
  const telegram = window.Telegram.WebApp;

  console.log(telegram);
  useEffect(() => {
    if (telegram) {
      telegram.ready();
    }
  }, []);

  const handleRequestPhone = () => {
    // درخواست شماره تلفن از کاربر
    telegram.sendData("request_phone_number");
  };

  return (
    <div>
      <button onClick={handleRequestPhone}>📞 ارسال شماره تلفن</button>
    </div>
  );
};

export default Auth;
