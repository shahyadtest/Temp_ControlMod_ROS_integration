import connectDB from "@/lib/db";
import RockPaperScissors from "@/models/RockPaperScissors";
import User from "@/models/User";
import jwt from "jsonwebtoken";
import { NextResponse } from "next/server";

export async function GET(req) {
  connectDB();

//   const { searchParams } = new URL(req.url);
//   const token = searchParams.get("userId");

  try {
    // استخراج توکن از کوکی یا هدر
    const token =
      req.cookies.get("token")?.value ||
      req.headers.get("authorization")?.split(" ")[1];

    if (!token) {
      return NextResponse.json({ error: "توکن یافت نشد." }, { status: 401 });
    }

    // استخراج userId از توکن
    let userId;
    try {
      const decoded = jwt.verify(token, process.env.JWT_SECRET);
      userId = decoded.userId;
    } catch (err) {
      return NextResponse.json({ error: "توکن نامعتبر است." }, { status: 401 });
    }

    // دریافت همه بازی‌های مربوط به کاربر
    const games = await RockPaperScissors.find({
      $or: [{ player1: userId }, { player2: userId }],
    }).sort({ createdAt: -1 });

    // ساخت آرایه تاریخچه
    const history = await Promise.all(
      games.map(async (game) => {
        const isPlayer1 = game.player1 === userId;
        const opponentId = isPlayer1 ? game.player2 : game.player1;

        const opponent = await User.findById(opponentId).select(
          "userName nickName _id"
        );

        return {
          gameId: game._id,
          gameName: "سنگ، کاغذ، قیچی",
          opponent,
          winner: game.winner,
          createdAt: game.createdAt,
        };
      })
    );

    return NextResponse.json({
      success: true,
      history,
    });
  } catch (error) {
    console.error("خطا در دریافت تاریخچه بازی:", error);
    return NextResponse.json({ error: "مشکلی رخ داده است." }, { status: 500 });
  }
}
