export const customPieces = () => {
  const pieceImages = {
    wP: "/pieces/white-pawn.png",
    wR: "/pieces/white-rook.png",
    wN: "/pieces/white-knight.png",
    wB: "/pieces/white-bishop.png",
    wQ: "/pieces/white-queen.png",
    wK: "/pieces/white-king.png",
    bP: "/pieces/black-pawn.png",
    bR: "/pieces/black-rook.png",
    bN: "/pieces/black-knight.png",
    bB: "/pieces/black-bishop.png",
    bQ: "/pieces/black-queen.png",
    bK: "/pieces/black-king.png",
  };

  const pieces = {};
  Object.keys(pieceImages).forEach((piece) => {
    pieces[piece] = ({ squareWidth }) => (
      <img
        src={pieceImages[piece]}
        alt={piece}
        style={{
          width: squareWidth,
          height: squareWidth,
          padding: "5px",
          objectFit: "contain",
        }}
      />
    );
  });

  return pieces;
};
