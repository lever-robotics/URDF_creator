import React, { createContext, useContext, useState, useImperativeHandle } from 'react';
import { URDFHistoryContext } from './URDFHistoryContext';
import { XMLtoScene, ScenetoXML } from '../../utils/SceneTransformUtils'

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
        if (currentScene) {
            const urdfXmlText = ScenetoXML(currentScene);
            historyContext.updateFromGUI(urdfXmlText);
        }
    };

    useImperativeHandle(historyContext.guiRef, () => ({
        updateURDFCode: (urdfXmlText) => {
            updateURDFScene(urdfXmlText);
        }
    }));

    return (
        <URDFGUIContext.Provider value={{ currentScene, updateURDFScene, saveURDFScene }}>
            {children}
        </URDFGUIContext.Provider>
    );
};
