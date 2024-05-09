import React, { useContext } from 'react';
import ThreeScene from './ThreeScene';
import ObjectParameters from './ObjectParameters';
import InsertToolBar from './InsertToolBar';
import { URDFContext } from '../URDFContext/URDFContext';

function URDFViewer() {
    const { urdf, updateURDF } = useContext(URDFContext);

    return (
        <div style={{ display: 'flex', width: '100vw', height: '100vh' }}>
            <ThreeScene urdf={urdf} updateURDF={updateURDF} />
        </div>
    );
}

export default URDFViewer;
