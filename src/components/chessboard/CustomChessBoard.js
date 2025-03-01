"use client";
import React, { useState } from "react";
import { Chessboard } from "react-chessboard";
import BoardLabels from "./BoardLabels";
import { Chess } from "chess.js";
import { getSquareColor } from "@/helper/helper";
import { customPieces } from "./customPieces";

const CustomChessBoard = () => {
  const [game, setGame] = useState(new Chess());
  const [highlightedSquares, setHighlightedSquares] = useState({});
  const [moveHistory, setMoveHistory] = useState([]);
  const [selectedSquare, setSelectedSquare] = useState(null);

  // get possible moves when user click on piece
  const getPossibleMoves = (square) => {
    const moves = game.moves({ square, verbose: true });
    const newHighlights = {};
    moves.forEach((move) => {
      newHighlights[move.to] = true;
    });
    setHighlightedSquares(newHighlights);
  };

  // انجام حرکت مهره با کلون کردن وضعیت بازی
  const makeMove = (from, to) => {
    // کلون کردن وضعیت فعلی بازی
    const newGame = new Chess(game.fen());
    const move = newGame.move({ from, to, promotion: "q" });
    if (move === null) {
      // اگر حرکت نامعتبر بود، هیچ کاری انجام نمی‌دهیم
      return false;
    }
    // آپدیت وضعیت بازی با نمونه جدید
    setGame(newGame);
    setSelectedSquare(null);
    setHighlightedSquares({});
    return true;
  };

  // مدیریت کلیک روی هر خانه
  const handleSquareClick = (square) => {
    // اگر یک مهره انتخاب شده و خانه کلیک‌شده جزو حرکات مجاز است
    if (selectedSquare && highlightedSquares[square]) {
      const success = makeMove(selectedSquare, square);
      if (!success) {
        // در صورت نامعتبر بودن حرکت (که نباید رخ بده چون از chess.js استفاده می‌کنیم)
        console.error("حرکت نامعتبر");
      }
      return;
    }

    // دریافت مهره موجود در خانه کلیک شده
    const piece = game.get(square);
    // اگر مهره وجود داشته باشد و به نوبت بازی باشد
    if (piece && piece.color === game.turn()) {
      // اگر روی همان مهره دوباره کلیک کردیم، انتخاب لغو شود
      if (square === selectedSquare) {
        setSelectedSquare(null);
        setHighlightedSquares({});
        return;
      }
      setSelectedSquare(square);
      getPossibleMoves(square);
    } else {
      // اگر خانه خالی یا متعلق به حریف است، انتخاب قبلی پاک شود
      setSelectedSquare(null);
      setHighlightedSquares({});
    }
  };

  return (
    <div className="w-full relative p-4 bg-gray-50 bg-opacity-15 backdrop-blur border border-gray-50 border-opacity-15 rounded-3xl">
      {/* board Labels */}
      <BoardLabels />

      <Chessboard
        animationDuration={300}
        onSquareClick={handleSquareClick}
        onPieceDrop={(sourceSquare, targetSquare) => {
          const success = makeMove(sourceSquare, targetSquare);
          return success;
        }}
        customPieces={customPieces()}
        customDarkSquareStyle={{ backgroundColor: "#373855" }}
        customLightSquareStyle={{ backgroundColor: "#f3f4f6" }}
        showBoardNotation={false}
        customBoardStyle={{
          height: "fit-content",
          borderRadius: "16px",
          direction: "ltr",
        }}
        position={game.fen()}
        customSquareStyles={{
          ...Object.fromEntries(
            Object.keys(highlightedSquares).map((square) => [
              square,
              {
                background: "radial-gradient(#3D4AEB 30%, transparent 30%)",
                borderRadius: "50%",
                // border: `16px solid ${
                //   getSquareColor(square) === "white" ? "#f3f4f6" : "#373855"
                // }`,
                // borderRadius: "100%",
              },
            ])
          ),
        }}
        id="BasicBoard"
      />
    </div>
  );
};

export default CustomChessBoard;
