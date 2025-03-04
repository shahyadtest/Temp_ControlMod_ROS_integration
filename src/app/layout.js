import "./globals.css";
import "../css/styles.css";
import { Providers } from "./providers";
import NextTopLoader from "nextjs-toploader";

export const metadata = {
  title: "Chess Game",
};

export default async function RootLayout({ children }) {
  return (
    <html lang="fa" dir="rtl" className="dark">

      <body className="w-full bg-blackColor">
        {/* <NextTopLoader color="#dc2626" showSpinner={false} /> */}

        <Providers>{children}</Providers>
      </body>
    </html>
  );
}
