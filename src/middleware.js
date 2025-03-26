import { NextResponse } from "next/server";

export function middleware(request) {
  const token = request.cookies.get("token")?.value;
  const currentPath = request.nextUrl.pathname;

  if (token || currentPath === "/auth") {
    return NextResponse.next();
  }

  const url = new URL(request.url);
  url.pathname = "/auth";
  return NextResponse.redirect(url.toString());
}

export const config = {
  matcher: ["/((?!_next|static|favicon.ico|assets|api|.*\\..*).*)"],
};
