#!/bin/sh

set -eu

manifest=/tmp/slides-export-manifest

case "${HOST_UID}:${HOST_GID}" in
  *[!0-9:]* | :*)
    echo "HOST_UID and HOST_GID must be numeric." >&2
    exit 1
    ;;
esac

awk '
  $1 == "-" && $2 == "Day:" {
    day = $3
    next
  }
  day != "" && $1 ~ /^[0-9]+:$/ {
    slide_id = $1
    sub(/:$/, "", slide_id)
    print day, slide_id, $2
  }
' /workspace/export_list.yaml > "${manifest}"

if [ ! -s "${manifest}" ]; then
  echo "No slides found in export_list.yaml." >&2
  exit 1
fi

mkdir -p /output

while read -r day slide_id html_path; do
  input_path="/workspace/${html_path}"
  output_dir="/output/Day${day}"
  filename="${html_path##*/}"
  deck_name="${filename%.html}"
  output_path="${output_dir}/${slide_id}_${deck_name}.pdf"

  if [ ! -f "${input_path}" ]; then
    echo "Slide source not found: ${html_path}" >&2
    exit 1
  fi

  mkdir -p "${output_dir}"
  echo "Exporting ${html_path} to ${output_path}"

  node /decktape/decktape.js \
    --chrome-path /usr/bin/chromium-browser \
    --chrome-arg=--no-sandbox \
    --chrome-arg=--disable-gpu \
    --chrome-arg=--allow-file-access-from-files \
    --chrome-arg=--disable-web-security \
    remark \
    "${input_path}" \
    "${output_path}"

  chown "${HOST_UID}:${HOST_GID}" "${output_dir}" "${output_path}"
done < "${manifest}"

chown "${HOST_UID}:${HOST_GID}" /output
echo "Slides exported to /output."
