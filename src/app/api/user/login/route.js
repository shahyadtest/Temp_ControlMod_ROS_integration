import { compare } from "bcryptjs";
import { NextResponse } from "next/server";
import connectDB from "@/lib/db";
import User from "@/models/User";
import { serialize } from "cookie";
import jwt from "jsonwebtoken";

export async function POST(req) {
  try {
    await connectDB();

    const { userName, password } = await req.json();

    if (!userName || !password) {
      return NextResponse.json(
        { message: "لطفاً نام کاربری و رمز عبور را وارد کنید." },
        { status: 400 }
      );
    }

    const user = await User.findOne({ userName });
    if (!user) {
      return NextResponse.json(
        { message: "کاربری با این نام کاربری یافت نشد." },
        { status: 400 }
      );
    }

    const isMatch = await compare(password, user.password);
    if (!isMatch) {
      return NextResponse.json(
        { message: "رمز عبور نادرست است." },
        { status: 400 }
      );
    }

    const accessToken = jwt.sign(
      { userId: user._id, userName: user.userName },
      process.env.JWT_SECRET
    );

    const cookie = serialize("token", accessToken, {
      httpOnly: true,
      secure: process.env.NODE_ENV === "production",
      sameSite: process.env.NODE_ENV === "production" ? "strict" : "lax",
      path: "/",
    });

    const response = NextResponse.json(
      {
        message: "ورود با موفقیت انجام شد",
        user: { ...user._doc },
      },
      { status: 200 }
    );

    response.headers.set("Set-Cookie", cookie);

    return response;
  } catch (error) {
    console.error("خطا در ورود:", error);
    return NextResponse.json({ error: "مشکلی رخ داده است." }, { status: 500 });
  }
}
