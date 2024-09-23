import { openDB } from 'idb';


// Function to open the database
export const openDatabase = async () => {
    return await openDB('stlFilesDB', 1, {
        upgrade(db) {
            if (!db.objectStoreNames.contains('files')) {
                db.createObjectStore('files', { keyPath: 'name' });
            }
        }
    });
};

// Function to get a file from the database
export const getFile = async (fileName: string) => {
    const db = await openDatabase();
    const transaction = db.transaction('files', 'readonly');
    const store = transaction.objectStore('files');
    const entry = await store.get(fileName);
    await transaction.done;
    return entry.file;
};

// Function to convert a Blob/File to an ArrayBuffer
export const blobToArrayBuffer = (blob: Blob) => {
    return new Promise<string>((resolve, reject) => {
        const reader = new FileReader();
        reader.onloadend = () => resolve(reader.result as string);
        reader.onerror = reject;
        reader.readAsArrayBuffer(blob);
    });
};