import React, { createContext, useContext, useState, useImperativeHandle, forwardRef } from 'react';
import { URDFHistoryContext } from './URDFHistoryContext';
import { ScenetoXML } from '../../utils/ScenetoXML';
import { XMLtoScene } from '../../utils/XMLtoScene';

export const URDFGUIContext = createContext();

export const URDFGUIProvider = forwardRef(({ children }, ref) => {
    const [currentScene, setCurrentScene] = useState(null);
    const historyContext = useContext(URDFHistoryContext);

    const updateURDFScene = (urdfXmlText) => {
        const newScene = XMLtoScene(urdfXmlText);
        setCurrentScene(newScene);
    };

    const saveURDFScene = (scene) => {
        console.log("Testing for scene");
        console.log(scene);
        if (scene) {
            console.log("Convert Scene to XML");
            const urdfXmlText = ScenetoXML(scene);
            console.log("XML is: ", urdfXmlText);
            console.log("Updating history and Code Contexts");
            historyContext.updateFromGUI(urdfXmlText);
        }
    };

    useImperativeHandle(historyContext.guiRef, () => ({
        updateURDFScene: (urdfXmlText) => {
            updateURDFScene(urdfXmlText);
        }
    }));

    return (
        <URDFGUIContext.Provider value={{ currentScene, updateURDFScene, saveURDFScene, setCurrentScene }}>
            {children}
        </URDFGUIContext.Provider>
    );
});

URDFGUIProvider.displayName = 'URDFGUIProvider';
