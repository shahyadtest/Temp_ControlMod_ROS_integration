import jwt from "jsonwebtoken";
import { NextResponse } from "next/server";
import connectDB from "@/lib/db";
import User from "@/models/User";

export async function GET(req) {
  try {
    await connectDB();

    // get user token
    const token =
      req.cookies.get("token")?.value ||
      req.headers.get("authorization").split(" ")[1];

    if (!token) {
      return NextResponse.json(
        { error: "وارد حساب کاربری شوید." },
        { status: 401 }
      );
    }

    // check user token
    const decoded = jwt.verify(token, process.env.JWT_SECRET);
    const user = await User.findById(decoded.userId).select("-password -__v");

    return NextResponse.json({ user }, { status: 200 });
  } catch (error) {
    return NextResponse.json({ error: "توکن نامعتبر است." }, { status: 401 });
  }
}
