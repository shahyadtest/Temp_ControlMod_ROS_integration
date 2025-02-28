/** @type {import('next').NextConfig} */
const nextConfig = {
  reactStrictMode: false,
  images: {
    unoptimized: true,
    remotePatterns: [
      {
        protocol: "https",
        hostname: "api.markazyab.ir",
        port: "",
      },
    ],
  },
};

export default nextConfig;
