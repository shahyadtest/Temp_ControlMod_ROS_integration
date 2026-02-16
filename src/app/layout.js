import "./globals.css";
import "../css/styles.css";
import { Providers } from "./providers";
import NextTopLoader from "nextjs-toploader";
import QueryProvider from "./QueryProvider";
import GetUserInfo from "@/lib/GetUserInfo";
import { cookies } from "next/headers";
import { baseURL } from "@/services/API";
import { getUser } from "@/lib/fetchUser";
import SocketContext from "@/context/SocketContext";

export const metadata = {
  title: "Gamehub",
};

export default async function RootLayout({ children }) {
  const userInfo = await getUser();

  return (
    <html lang="fa" dir="rtl" className="dark">
      <body className="w-full bg-blackColor">
        <NextTopLoader color="#3D4AEB" showSpinner={false} />

        <Providers>
          <GetUserInfo userInfo={userInfo} />

          <QueryProvider>
            <SocketContext userInfo={userInfo}>{children}</SocketContext>
          </QueryProvider>
        </Providers>
      </body>
    </html>
  );
}
