import React, { createContext, useContext, useState, useImperativeHandle } from 'react';
import { URDFHistoryContext } from './URDFHistoryContext';
import { ScenetoXML } from '../../utils/ScenetoXML'
import { XMLtoScene } from '../../utils/XMLtoScene'

export const URDFGUIContext = createContext();

export const URDFGUIProvider = ({ children }) => {
    const [currentScene, setCurrentScene] = useState(null); // Initially, there's no scene
    const historyContext = useContext(URDFHistoryContext);

    // Updates the current state of the Scene in the GUI context only
    const updateURDFScene = (urdfXmlText) => {
        const newScene = XMLtoScene(urdfXmlText);
        setCurrentScene(newScene);
    };

    // Saves the current Scene state to the history and updates the code provider
    const saveURDFScene = () => {
        console.log("Testing for scene");
        if (currentScene) {
            console.log("Convert Scene to XML");
            const urdfXmlText = ScenetoXML(currentScene);
            console.log("XML is: ", urdfXmlText);
            console.log("Updating history and Code Contexts");
            historyContext.updateFromGUI(urdfXmlText);
        }
    };

    useImperativeHandle(historyContext.guiRef, () => ({
        updateURDFCode: (urdfXmlText) => {
            updateURDFScene(urdfXmlText);
        }
    }));

    return (
        <URDFGUIContext.Provider value={{ currentScene, updateURDFScene, saveURDFScene, setCurrentScene }}>
            {children}
        </URDFGUIContext.Provider>
    );
};
