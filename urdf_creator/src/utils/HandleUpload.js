

export function handleUpload(file){

    // let content = null;

    const reader = new FileReader();
    reader.onload = (e) => {
        console.log(e.target.result);
    }
    reader.readAsText(file);
}