// node.js
const mume = require("@shd101wyy/mume");

// es6
// import * as mume from "@shd101wyy/mume"

async function main(chrome_path, file_in) {
  await mume.init();

  const engine = new mume.MarkdownEngine({
    filePath: file_in,
    config: {
      previewTheme: "github-light.css",
      // revealjsTheme: "white.css"
      codeBlockTheme: "default.css",
      printBackground: true,
      enableScriptExecution: true, // <= for running code chunks
      chromePath: chrome_path,
    },
  });

  await engine.chromeExport({ fileType: "pdf", runAllCodeChunks: true }); // fileType = 'pdf'|'png'|'jpeg'

  return process.exit();
}

main(process.argv[2], process.argv[3]);