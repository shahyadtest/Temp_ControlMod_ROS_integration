import "./globals.css";
import "../css/styles.css";
import { Providers } from "./providers";
import NextTopLoader from "nextjs-toploader";
import Script from "next/script";

export const metadata = {
  title: "Chess Game",
};

export default async function RootLayout({ children }) {
  return (
    <html lang="fa" dir="rtl" className="dark">
      <head>
        {/* لود کردن Telegram Web App API */}
        <Script
          src="https://telegram.org/js/telegram-web-app.js"
          // strategy="beforeInteractive"
        />
      </head>

      <body className="w-full bg-blackColor">
        {/* <NextTopLoader color="#dc2626" showSpinner={false} /> */}

        <Providers>{children}</Providers>
      </body>
    </html>
  );
}
