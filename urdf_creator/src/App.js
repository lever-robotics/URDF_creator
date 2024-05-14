import React, { useRef } from 'react';
import DownloadRobotPackage from './components/DownloadRobotPackage/DownloadRobotPackage';
import ThreeScene from './components/ThreeScene/ThreeScene';
import { URDFHistoryProvider } from './components/URDFContext/URDFHistoryContext';
import { URDFGUIProvider } from './components/URDFContext/URDFGUIContext';
import { URDFCodeProvider } from './components/URDFContext/URDFCodeContext';
import URDFCodeDisplayer from './components/CodeDisplayer/URDFCodeDisplayer';

const App = () => {
  const guiRef = useRef();
  const codeRef = useRef();

  return (
    <URDFHistoryProvider>
      <URDFGUIProvider ref={guiRef}>
        <ThreeScene />
      </URDFGUIProvider>
      <URDFCodeProvider ref={codeRef}>
        <DownloadRobotPackage />
        <URDFCodeDisplayer />
      </URDFCodeProvider>
    </URDFHistoryProvider>
  );
};

export default App;
