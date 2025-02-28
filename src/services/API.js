import axios from "axios";

export const baseURL = "https://api.markazyab.ir/api";
export const siteURL = "https://api.markazyab.ir";

// axios config
const servicesApi = axios.create({
  baseURL: "https://api.markazyab.ir/api",
  withCredentials: false,
  timeout: 60000000,
  headers: {
    "Content-Type": "application/json",
  },
});

// post method
const postData = async (
  param,
  data,
  onUploadProgress,
  headers,
  withToken = true
) => {
  if (withToken) {
    let token = localStorage.getItem("token");
    const res = await servicesApi.post(param, data, {
      headers: {
        Authorization: `Bearer ${token}`,
        ...(headers === "multipart" && {
          "Content-Type": "multipart/form-data",
        }),
      },
      onUploadProgress,
    });

    return res;
  }

  const res = await servicesApi.post(param, data);
  return res;
};

// get method
const getData = async (param, data, headers, withToken = true) => {
  if (withToken) {
    let token = localStorage.getItem("token");

    const res = await servicesApi.get(param, {
      params: data,
      headers: {
        Authorization: `Bearer ${token}`,
      },
    });

    return res;
  }

  const res = await servicesApi.get(param, { params: data });
  return res;
};

// patch method
const patchData = async (param, data, withToken = true) => {
  if (withToken) {
    let token = localStorage.getItem("token");

    const res = await servicesApi.patch(param, data, {
      headers: { Authorization: `Bearer ${token}` },
    });

    return res;
  }

  const res = await servicesApi.patch(param, data);
  return res;
};

// patch method
const putData = async (param, data, withToken = true) => {
  if (withToken) {
    let token = localStorage.getItem("token");

    const res = await servicesApi.put(param, data, {
      headers: { Authorization: `Bearer ${token}` },
    });

    return res;
  }

  const res = await servicesApi.put(param, data);
  return res;
};

// delete method
const deleteData = async (param, data, withToken = true) => {
  if (withToken) {
    let token = localStorage.getItem("token");

    const res = await servicesApi.delete(param, {
      headers: { Authorization: `Bearer ${token}` },
      data: data,
    });

    return res;
  }

  const res = await servicesApi.delete(param, data);
  return res;
};

export { postData, getData, patchData, deleteData, putData };
