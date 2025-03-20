import "./globals.css";
import "../css/styles.css";
import { Providers } from "./providers";
import NextTopLoader from "nextjs-toploader";
import GetUser from "@/lib/GetUser";

export const metadata = {
  title: "Chess Game",
};

export default async function RootLayout({ children }) {


  return (
    <html lang="fa" dir="rtl" className="dark">
      <body className="w-full bg-blackColor">
        <NextTopLoader color="#3D4AEB" showSpinner={false} />


        <Providers>
          <GetUser />

        {children}
        </Providers>
      </body>
    </html>
  );
}
