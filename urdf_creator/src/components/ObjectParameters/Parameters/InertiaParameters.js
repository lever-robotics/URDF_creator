import React, { useState, useEffect } from 'react';
import Inertia from '../../../Models/Inertia';

function InertiaParameters({ selectedObject, setInertia }) {
    const [Ixx, setIxx] = useState('');
    const [Ixy, setIxy] = useState('');
    const [Ixz, setIxz] = useState('');
    const [Iyy, setIyy] = useState('');
    const [Iyz, setIyz] = useState('');
    const [Izz, setIzz] = useState('');

    useEffect(() => {
        if (selectedObject) {
            setIxx(selectedObject.userData.inertia.ixx || '');
            setIxy(selectedObject.userData.inertia.ixy || '');
            setIxz(selectedObject.userData.inertia.ixz || '');
            setIyy(selectedObject.userData.inertia.iyy || '');
            setIyz(selectedObject.userData.inertia.iyz || '');
            setIzz(selectedObject.userData.inertia.izz || '');
        }
    }, [JSON.stringify(selectedObject.userData.inertia)]);

    const handleInertiaChange = (prop, value) => {
        const newValue = parseFloat(value);
        if (isNaN(newValue)) return;
        const inertia = new Inertia(selectedObject.userData.inertia.mass, parseFloat(Ixx), parseFloat(Iyy), parseFloat(Izz), parseFloat(Ixy), parseFloat(Ixz), parseFloat(Iyz));
        inertia.customInertia = true;
        setInertia(selectedObject, inertia);
    };

    return (
        <div>
            <strong>Moment of Inertia:</strong>
            <ul>
                <li>
                    Ixx:
                    <input
                        type="number"
                        value={Ixx}
                        onChange={(e) => setIxx(e.target.value)}
                        onBlur={() => handleInertiaChange('Ixx', Ixx)}
                    />
                </li>
                <li>
                    Ixy:
                    <input
                        type="number"
                        value={Ixy}
                        onChange={(e) => setIxy(e.target.value)}
                        onBlur={() => handleInertiaChange('Ixy', Ixy)}
                    />
                </li>
                <li>
                    Ixz:
                    <input
                        type="number"
                        value={Ixz}
                        onChange={(e) => setIxz(e.target.value)}
                        onBlur={() => handleInertiaChange('Ixz', Ixz)}
                    />
                </li>
                <li>
                    Iyy:
                    <input
                        type="number"
                        value={Iyy}
                        onChange={(e) => setIyy(e.target.value)}
                        onBlur={() => handleInertiaChange('Iyy', Iyy)}
                    />
                </li>
                <li>
                    Iyz:
                    <input
                        type="number"
                        value={Iyz}
                        onChange={(e) => setIyz(e.target.value)}
                        onBlur={() => handleInertiaChange('Iyz', Iyz)}
                    />
                </li>
                <li>
                    Izz:
                    <input
                        type="number"
                        value={Izz}
                        onChange={(e) => setIzz(e.target.value)}
                        onBlur={() => handleInertiaChange('Izz', Izz)}
                    />
                </li>
            </ul>
        </div>
    );
}

export default InertiaParameters;
