"use client";
import React, { useState } from "react";
import { Chessboard } from "react-chessboard";
import BoardLabels from "./BoardLabels";
import { Chess } from "chess.js";
import { getSquareColor } from "@/helper/helper";
import { blackPieces, customPieces, whitePieces } from "./customPieces";
import Image from "next/image";

const CustomChessBoard = () => {
  const [game, setGame] = useState(new Chess());
  const [highlightedSquares, setHighlightedSquares] = useState({});
  const [moveHistory, setMoveHistory] = useState([]);
  const [selectedSquare, setSelectedSquare] = useState(null);
  const [capturedPieces, setCapturedPieces] = useState([]);

  // get possible moves when user click on piece
  const getPossibleMoves = (square) => {
    const moves = game.moves({ square, verbose: true });
    return Object.fromEntries(
      moves.map((move) => [move.to, { backgroundColor: "#a9d18e" }]) // رنگ خانه‌های ممکن
    );
  };

  // انجام حرکت مهره با کلون کردن وضعیت بازی
  const makeMove = (from, to) => {
    const move = game.move({ from, to, promotion: "q" });

    if (move) {
      if (move.captured) {
        setCapturedPieces((prev) => [
          ...prev,
          { type: move.captured, color: move.color === "w" ? "b" : "w" },
        ]);
      }

      setGame(new Chess(game.fen()));
      setSelectedSquare(null);
      setHighlightedSquares({});
    }
  };

  // مدیریت کلیک روی هر خانه
  const handleSquareClick = (square) => {
    if (selectedSquare === square) {
      setSelectedSquare(null);
      setHighlightedSquares({});
      return;
    }

    if (selectedSquare && highlightedSquares[square]) {
      makeMove(selectedSquare, square);
      return;
    }

    const piece = game.get(square);
    if (piece?.color === game.turn()) {
      setSelectedSquare((prev) => (prev === square ? null : square));
      setHighlightedSquares((prev) =>
        prev === square ? {} : getPossibleMoves(square)
      );
    } else {
      setSelectedSquare(null);
      setHighlightedSquares({});
    }
  };


  return (
    <div className="flex flex-col gap-3">
      <div className="flex items-center gap-1 self-end">
        {capturedPieces.map(
          (pieces) =>
            pieces.color === "b" && (
              <img src={blackPieces[pieces.type]} className="size-8" />
            )
        )}
      </div>

      <div className="w-full relative p-4 bg-gray-50 bg-opacity-15 backdrop-blur border border-gray-50 border-opacity-15 rounded-3xl">
        {/* board Labels */}
        <BoardLabels />

        <Chessboard
          animationDuration={500}
          onSquareClick={handleSquareClick}
          onPieceDrop={(sourceSquare, targetSquare) => {
            const success = makeMove(sourceSquare, targetSquare);
            return success;
          }}
          customPieces={customPieces()}
          customDarkSquareStyle={{ backgroundColor: "#373855" }}
          customLightSquareStyle={{ backgroundColor: "#d1d5db" }}
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
                },
              ])
            ),
          }}
          id="BasicBoard"
        />
      </div>

      <div className="flex items-center gap-1 self-start">
        {capturedPieces.map(
          (pieces) =>
            pieces.color === "w" && (
              <img src={whitePieces[pieces.type]} className="size-8" />
            )
        )}
      </div>
    </div>
  );
};

export default CustomChessBoard;
