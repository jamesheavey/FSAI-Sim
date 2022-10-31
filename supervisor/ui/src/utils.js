export const downloadFile = (body, filename, fileType = "text/text") => {
  const encodedUri = encodeURI(`data:${fileType};charset=utf-8,${body}`);
  const link = document.createElement("a");
  link.setAttribute("href", encodedUri);
  link.setAttribute("download", filename);
  document.body.appendChild(link); // Required for FF

  link.click();
};

export const downloadImage = (frame, filename) => {
  const data = `data:image/jpeg;base64,${frame}`
  const link = document.createElement("a");
  link.setAttribute("href", data);
  link.setAttribute("download", filename);
  document.body.appendChild(link); // Required for FF

  link.click();
}

export const downloadBlobFromUrl = (url, filename) => {
  fetch(url, {
    method: "GET",
    headers: {}
  })
    .then(response => {
      response.arrayBuffer().then(function(buffer) {
        const url = window.URL.createObjectURL(new Blob([buffer]));
        const link = document.createElement("a");
        link.href = url;
        link.setAttribute("download", filename);
        document.body.appendChild(link);
        link.click();
      });
    })
    .catch(err => {
      console.log(err);
    });
}

export const durationFormatter = (time_in_seconds, abrv = false) => {
  let unit = "";
  let value = 0;

  if (time_in_seconds < 60) {
    // Secs up to a minute
    value = Math.floor(time_in_seconds);
    unit = abrv ? "sec" : "second";
  } else if (time_in_seconds < 60 * 90) {
    // Mins up to 90 minutes
    value = Math.ceil(time_in_seconds / 60);
    unit = abrv ? "min" : "minute";
  } else if (time_in_seconds < 60 * 60 * 24) {
    // Hrs up to a day
    value = Math.ceil((time_in_seconds / (60 * 60)) * 10) / 10;
    unit = abrv ? "hr" : "hour";
  } else {
    // Otherwise, days
    value = Math.ceil((time_in_seconds / (60 * 60 * 24)) * 10) / 10;
    unit = abrv ? "dy" : "day";
  }

  return `${value} ${unit}${value === 1 ? "" : "s"} ago`;
};
