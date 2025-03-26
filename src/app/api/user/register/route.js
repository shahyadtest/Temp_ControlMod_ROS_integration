import { hash } from "bcryptjs";
import { NextResponse } from "next/server";
import connectDB from "@/lib/db";
import User from "@/models/User";
import { serialize } from "cookie";
import jwt from "jsonwebtoken";

export async function POST(req) {
  try {
    await connectDB();

    const { nickName, userName, phoneNumber, password } = await req.json();

    if (!nickName || !userName || !phoneNumber || !password) {
      return NextResponse.json(
        { error: "لطفاً همه فیلدها را پر کنید." },
        { status: 400 }
      );
    }

    // check phone number is exsit in db or note
    const existingUser = await User.findOne({ phoneNumber });
    if (existingUser) {
      return NextResponse.json(
        { error: "این شماره موبایل قبلاً ثبت شده است." },
        { status: 400 }
      );
    }

    // hash password
    const hashedPassword = await hash(password, 10);

    // create new user in db
    const newUser = await User.create({
      nickName,
      userName,
      phoneNumber,
      password: hashedPassword,
    });

    await newUser.save();

    // create accessToken without expire
    const accessToken = jwt.sign(
      { userId: newUser._id, phoneNumber: newUser.phoneNumber },
      process.env.JWT_SECRET
    );

    // save token in coockie(http only)
    const cookie = serialize("token", accessToken, {
      httpOnly: true,
      secure: process.env.NODE_ENV === "production",
      sameSite: process.env.NODE_ENV === "production" ? "strict" : "lax",
      path: "/",
    });

    const response = NextResponse.json(
      {
        message: "ثبت‌ نام با موفقیت انجام شد",
        user: { ...newUser._doc },
      },
      { status: 200 }
    );

    response.headers.set("Set-Cookie", cookie);

    return response;
  } catch (error) {
    console.error("خطا در ثبت‌نام:", error);
    return NextResponse.json({ error: "مشکلی رخ داده است." }, { status: 500 });
  }
}
