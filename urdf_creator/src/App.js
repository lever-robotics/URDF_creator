import React from 'react';
import DownloadRobotPackage from './components/DownloadRobotPackage/DownloadRobotPackage';
import ThreeScene from './components/ThreeScene/ThreeScene';
import { URDFHistoryProvider } from './components/URDFContext/URDFHistoryContext';
import { URDFGUIProvider } from './components/URDFContext/URDFGUIContext';
import { URDFCodeProvider } from './components/URDFContext/URDFCodeContext';
import URDFCodeDisplayer from './components/CodeDisplayer/URDFCodeDisplayer';

const App = () => {
  return (
    <URDFHistoryProvider>
      <URDFGUIProvider>
        <ThreeScene />
      </URDFGUIProvider>
      <URDFCodeProvider>
        <DownloadRobotPackage />
        <URDFCodeDisplayer />
      </URDFCodeProvider>
    </URDFHistoryProvider>
  );
};

export default App;
