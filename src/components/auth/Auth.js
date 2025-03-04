"use client";
import Script from "next/script";
import React, { useEffect } from "react";

const Auth = () => {
  useEffect(() => {
    if (typeof window !== "undefined" && window.Telegram?.WebApp) {
      window.Telegram.WebApp.ready(); // اطمینان از اینکه در مرورگر اجرا می‌شود
    }
  }, []);

  const handleRequestPhone = async () => {
    if (typeof window !== "undefined" && window.Telegram?.WebApp) {
      // ارسال درخواست شماره تلفن به ربات تلگرام
      const tg = window.Telegram.WebApp;
      tg.sendData("request_phone_number"); // باید از ربات برای درخواست استفاده کنید
    }
  };

  return (
    <div>
      <Script
        src="https://telegram.org/js/telegram-web-app.js"
        strategy="beforeInteractive" // لود کردن اسکریپت قبل از تعامل
      />

      <button onClick={handleRequestPhone}>📞 ارسال شماره تلفن</button>
    </div>
  );
};

export default Auth;
