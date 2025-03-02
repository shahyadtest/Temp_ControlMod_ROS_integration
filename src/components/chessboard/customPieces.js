export const whitePieces = {
  p: "/pieces/white-pawn.svg",
  r: "/pieces/white-rook.svg",
  n: "/pieces/white-knight.svg",
  b: "/pieces/white-bishop.svg",
  q: "/pieces/white-queen.svg",
  k: "/pieces/white-king.svg",
} 

export const blackPieces = {
  p: "/pieces/black-pawn.svg",
  r: "/pieces/black-rook.svg",
  n: "/pieces/black-knight.svg",
  b: "/pieces/black-bishop.svg",
  q: "/pieces/black-queen.svg",
  k: "/pieces/black-king.svg",
} 

export const customPieces = () => {
  const pieceImages = {
    wP: "/pieces/white-pawn.svg",
    wR: "/pieces/white-rook.svg",
    wN: "/pieces/white-knight.svg",
    wB: "/pieces/white-bishop.svg",
    wQ: "/pieces/white-queen.svg",
    wK: "/pieces/white-king.svg",
    bP: "/pieces/black-pawn.svg",
    bR: "/pieces/black-rook.svg",
    bN: "/pieces/black-knight.svg",
    bB: "/pieces/black-bishop.svg",
    bQ: "/pieces/black-queen.svg",
    bK: "/pieces/black-king.svg",
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
