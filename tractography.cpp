
// Created by xujuan sun on 11/18/19.
// step1: read the image (tensor type)
//step2: get the eigenValue(pixel by pixel), and visualize it.
//step3: try to get the track

#include "itkImage.h"
#include "itkDiffusionTensor3DReconstructionImageFilter.h"
#include "itkTensorFractionalAnisotropyImageFilter.h"
#include "itkTensorRelativeAnisotropyImageFilter.h"
#include "itkNrrdImageIO.h"
#include "itkImageSeriesReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkDiffusionTensor3D.h"
#include "itkImageRegionIterator.h"
#include "itkImageToVTKImageFilter.h"
#include <vtkVector.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkImageSliceMapper.h>
#include <vtkImageActor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkCamera.h>
#include <vtkImageProperty.h>
#include <vtkPointPicker.h>
#include <vtkRendererCollection.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkLine.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkActor.h>
#include <vtkUnsignedCharArray.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkCommand.h>
#include <vtkLineSource.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <limits>

vtkVector3d get_next_point(vtkVector3d cur_point, vtkSmartPointer <vtkImageData> im, double step_size);
vtkSmartPointer <vtkPolyData> create_line_poly_data(const std::vector<vtkVector3d> & end_point_list, unsigned char color_rgb []);
void render_trajectory_with_timer(vtkSmartPointer<vtkPolyData> trajectory_line, unsigned char colr_rgb []);

class TimerCallbackGrow : public vtkCommand
{
public:
    TimerCallbackGrow ()
    {
    }

    ~TimerCallbackGrow()
    {
    }

    static TimerCallbackGrow *New()
    {
        TimerCallbackGrow *callback = new TimerCallbackGrow() ;
        return callback ;
    }

    vtkSmartPointer<vtkPolyData> get_traction_n (int n)
    {
        std::vector<vtkVector3d> trajectory;

        vtkVector3d init_voxel = vtkVector3d(_seed_point[0], _seed_point[1], _seed_point[2]);
        trajectory.push_back(init_voxel);

        double step_size = _step_size;
        int total_itr = 1000;
        vtkVector3d pre_point = init_voxel;
        for (int i = 0; i < n+1; i++)
        {
            vtkVector3d next_point = get_next_point(pre_point, _im, step_size);
            std::cout << "Next point: " << next_point[0] << " " << next_point[1] << " " << next_point[2] << std::endl;
            trajectory.push_back(next_point);
            pre_point = next_point;
        }
        vtkSmartPointer<vtkPolyData> trajectory_line = create_line_poly_data(trajectory, _color_rgb);

        return trajectory_line;
    }

    virtual void Execute ( vtkObject *caller, unsigned long eventId, void *callData )
    {
        if (_cur_pt_id < 1000)
        {
            vtkSmartPointer<vtkPolyData> cur_traj = get_traction_n(_cur_pt_id);

            vtkSmartPointer<vtkPolyDataMapper> mapper = dynamic_cast<vtkPolyDataMapper *> (_actor->GetMapper());
            mapper->SetInputData(cur_traj);
            _render_window->Render();
        }
        _cur_pt_id++;
    }

    void SetGrowTimerData(unsigned char color_rgb [3], vtkVector3d seed_point, double step_size, vtkSmartPointer<vtkImageData> im)
    {
        _color_rgb[0] = color_rgb[0];
        _color_rgb[1] = color_rgb[1];
        _color_rgb[2] = color_rgb[2];
        _seed_point = seed_point;
        _step_size = step_size;
        _im = im;
    }

    void SetRendering(vtkSmartPointer<vtkRenderWindow> render_window, vtkSmartPointer<vtkActor> actor)
    {
        _render_window = render_window;
        _actor = actor;
    }

private:
    int _cur_pt_id = 0;
    unsigned char _color_rgb[3];
    vtkVector3d _seed_point;
    double _step_size;
    vtkSmartPointer<vtkImageData> _im;
    vtkSmartPointer<vtkRenderWindow> _render_window;
    vtkSmartPointer<vtkActor> _actor;
} ;

class MyMouseCallbackInteractorStyle : public vtkInteractorStyleImage
{
private:
    int _seed_pt_count = 0;
    int _num_colors = 3;
    std::vector<std::vector<unsigned char>> _colors;

private:
    void set_colors()
    {
        _colors.push_back(std::vector<unsigned char>{255,  0,  0});
        _colors.push_back(std::vector<unsigned char>{  0,255,  0});
        _colors.push_back(std::vector<unsigned char>{  0,  0,255});
    }

public:
    MyMouseCallbackInteractorStyle() { set_colors(); }
    ~MyMouseCallbackInteractorStyle() {}

    static MyMouseCallbackInteractorStyle* New()
    {
        MyMouseCallbackInteractorStyle *callback = new MyMouseCallbackInteractorStyle() ;
        return callback ;
    }

    virtual void OnRightButtonDown() override
    {
        int x, y ;
        this->Interactor->GetEventPosition ( x, y ) ;
        // std::cout << "Picked pixel: " << x << " " << y << std::endl ;

        vtkSmartPointer < vtkPointPicker > picker = dynamic_cast < vtkPointPicker *> ( this->Interactor->GetPicker() ) ;
        // this->Interactor->GetPicker()->Pick(x, y, 0,
        vtkSmartPointer <vtkRenderWindow> render_window = this->Interactor->GetRenderWindow();
        vtkSmartPointer <vtkRenderer> renderer = render_window->GetRenderers()->GetFirstRenderer();
        picker->Pick(x, y, 0, renderer);

        double picked[3];
        picker->GetPickPosition(picked);
        std::cout << "Picked voxel location: " << picked[0] << " " << picked[1] << " " << picked[2] << std::endl;

        vtkSmartPointer<vtkImageSliceMapper> mapper = dynamic_cast<vtkImageSliceMapper*> (picker->GetMapper());
        if (mapper != NULL)
        {
            unsigned char color [3] = {0, 0, 0};
            color[0] = _colors[_seed_pt_count][0];
            color[1] = _colors[_seed_pt_count][1];
            color[2] = _colors[_seed_pt_count][2];
            double color_fl [3] = {0, 0, 0};
            color_fl[0] = (double) color[0] / 255.0;
            color_fl[1] = (double) color[1] / 255.0;
            color_fl[2] = (double) color[2] / 255.0;

            vtkSmartPointer <vtkImageData> im = mapper->GetInput();

            double vect_val_x = im->GetScalarComponentAsDouble((int)picked[0], (int)picked[1], (int)picked[2], 0);
            double vect_val_y = im->GetScalarComponentAsDouble((int)picked[0], (int)picked[1], (int)picked[2], 1);
            double vect_val_z = im->GetScalarComponentAsDouble((int)picked[0], (int)picked[1], (int)picked[2], 2);

            std::cout << std::endl;
            std::cout << "Vector val at voxel " << vect_val_x << " " << vect_val_y << " " << vect_val_z << std::endl;

            // Draw seeds point as sphere.
            vtkSmartPointer<vtkSphereSource> sphere_source = vtkSmartPointer<vtkSphereSource>::New();
            sphere_source->SetCenter(picked[0], picked[1], picked[2]);
            sphere_source->SetRadius(0.8);
            sphere_source->Update();
            vtkSmartPointer<vtkPolyDataMapper> sphere_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            sphere_mapper->SetInputData(sphere_source->GetOutput());
            vtkSmartPointer<vtkActor> sphere_actor = vtkSmartPointer<vtkActor>::New();
            sphere_actor->SetMapper(sphere_mapper);
            sphere_actor->GetProperty()->SetColor(color_fl);
            renderer->AddActor(sphere_actor);
            render_window->Render();
            vtkVector3d init_voxel = vtkVector3d(picked[0], picked[1], picked[2]);
            vtkSmartPointer<vtkPolyDataMapper> line_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            vtkSmartPointer<vtkPolyData> null_polydata = vtkSmartPointer<vtkPolyData>::New();
            line_mapper->SetInputData(null_polydata);
            vtkSmartPointer<vtkActor> line_actor = vtkSmartPointer<vtkActor>::New();
            line_actor->SetMapper(line_mapper);
            line_actor->GetProperty()->SetLineWidth(5);
            renderer->AddActor(line_actor);
            double step_size = 0.1;
            vtkSmartPointer<TimerCallbackGrow> cb = vtkSmartPointer<TimerCallbackGrow>::New();
            cb->SetGrowTimerData(color, init_voxel, step_size, im);
            cb->SetRendering(render_window, line_actor);
            this->Interactor->AddObserver(vtkCommand::TimerEvent, cb);
            this->_seed_pt_count++;
        }

        vtkInteractorStyleImage::OnRightButtonDown();
    }
};

int main(int argc, char * argv[])
{
    constexpr unsigned int Dimension = 3;
    using GradientPixelType = itk::Vector<float,Dimension>;
    unsigned int   numberOfImages = 0;
    unsigned int   numberOfGradientImages = 0;
    bool    readb0 = false;
    double  b0 = 0;
    using PixelType = unsigned short;
    using TensorType = itk::DiffusionTensor3D<float>;
    using ImageType = itk::Image<TensorType, 3>;
    itk::ImageFileReader<ImageType>::Pointer reader = itk::ImageFileReader<ImageType>::New();
    ImageType::Pointer img;
    reader->SetFileName(argv[1]);
    img = reader->GetOutput();
    ImageType::RegionType region = img->GetLargestPossibleRegion();
    ImageType::SizeType size = region.GetSize();
    using GradientImageType = itk::Image<GradientPixelType,Dimension>;
    GradientImageType::IndexType start_gd;
    start_gd[0] = 0;
    start_gd[1] = 0;
    start_gd[2] = 0;
    GradientImageType::SizeType size_gd;
    size_gd[0] = size[0];
    size_gd[1] = size[1];
    size_gd[2] = size[2];
    GradientImageType::RegionType region_gd;
    region_gd.SetSize(size_gd);
    region_gd.SetIndex(start_gd);
    GradientImageType::Pointer image_gd = GradientImageType::New();
    image_gd->SetRegions(region_gd);
    image_gd->Allocate();
    using FractionalAnisotropyImageType = itk::Image<float, Dimension>;
    FractionalAnisotropyImageType::Pointer image_fa = FractionalAnisotropyImageType::New();
    image_fa->SetRegions(region_gd);
    image_fa->Allocate();
    itk::ImageRegionIterator<ImageType> it_ori(img, img->GetLargestPossibleRegion());
    itk::ImageRegionIterator<GradientImageType> it_gd(image_gd, image_gd->GetLargestPossibleRegion());
    itk::ImageRegionIterator<FractionalAnisotropyImageType> it_fa(image_fa, image_fa->GetLargestPossibleRegion());
    int counter = 0;
    while (!it_ori.IsAtEnd())
    {
        TensorType diffusion_tensor = it_ori.Get();
        TensorType::EigenValuesArrayType eigenvalues;
        TensorType::EigenVectorsMatrixType eigenvectors;
        diffusion_tensor.ComputeEigenAnalysis( eigenvalues, eigenvectors);
        int max_idx = 0;
        float max_eigen_val = 0;
        for (int eigen_idx = 0; eigen_idx < Dimension; eigen_idx++)
        {
            if (eigenvalues[eigen_idx] > max_eigen_val)
            {
                max_eigen_val = eigenvalues[eigen_idx];
                max_idx = eigen_idx;
            }
        }

        GradientPixelType voxel_vect;
        for (int dim = 0; dim < Dimension; dim ++)
            voxel_vect[dim] = 0;

        if (max_eigen_val > 0)
        {
            for (int dim = 0; dim < Dimension; dim ++)
                voxel_vect[dim] = std::abs(eigenvectors[max_idx][dim]) * diffusion_tensor.GetFractionalAnisotropy();
        }
        it_gd.Set(voxel_vect);
        it_fa.Set(diffusion_tensor.GetFractionalAnisotropy());
        ++it_ori;
        ++it_gd;
        ++it_fa;
    }
    typedef itk::ImageToVTKImageFilter<GradientImageType> ConnectorType ;
    ConnectorType::Pointer connector = ConnectorType::New() ;
    connector->SetInput ( image_gd ) ;
    connector->Update() ;

    vtkSmartPointer <vtkImageData> vtkImage = connector->GetOutput();

    // mapper???
    vtkSmartPointer < vtkImageSliceMapper > mapper = vtkSmartPointer < vtkImageSliceMapper >::New() ;
    mapper->SetInputData ( vtkImage ) ;
    //mapper->SetSliceNumber ( 100 ) ;
    //mapper->SetOrientationToZ() ;
    mapper->SliceAtFocalPointOn() ;
    mapper->SliceFacesCameraOn() ;

    // Image property
    vtkSmartPointer < vtkImageProperty > image_property = vtkSmartPointer <vtkImageProperty>::New();
    image_property->SetColorWindow(1.0);
    image_property->SetColorLevel(0.5);

    // actor????
    vtkSmartPointer < vtkImageActor > actor = vtkSmartPointer < vtkImageActor >::New() ;
    actor->SetMapper ( mapper ) ;
    actor->SetProperty(image_property);
    actor->InterpolateOff();

    // create scene
    vtkSmartPointer < vtkRenderer > renderer = vtkSmartPointer < vtkRenderer >::New() ;
    renderer->AddActor ( actor ) ;
    renderer->SetBackground ( 1, 1, 1 ) ;
    // Picker to pick pixels
    // vtkSmartPointer<vtkPropPicker> propPicker = vtkSmartPointer<vtkPropPicker>::New();
    // propPicker->PickFromListOn();

    // Give the picker a prop to pick
    // vtkImageActor* imageActor = imageViewer->GetImageActor();
    // propPicker->AddPickList(actor);

    // our camera is not facing the right way, we need to fix it
    vtkSmartPointer < vtkCamera > camera = renderer->GetActiveCamera () ;
    // get extent = size in voxels
    // get spacing = voxel size
    // get origin
    int extent[6] ;
    vtkImage->GetExtent ( extent ) ;
    double spacing[3] ;
    vtkImage->GetSpacing ( spacing ) ;
    double origin[3] ;
    vtkImage->GetOrigin ( origin ) ;
    double centerImage[3] ;
    centerImage[0] = origin[0] + 0.5 * ( extent[0] + extent[1] + 1) * spacing[0] ;
    centerImage[1] = origin[1] + 0.5 * ( extent[2] + extent[3] + 1) * spacing[1] ;
    centerImage[2] = origin[2] + 0.5 * ( extent[4] + extent[5]) * spacing[2] ;
    std::cout << extent[0] << " " << extent[1] << " " << extent[2] << " " << extent[3] << " " << extent[4] << " " << extent[5] << std::endl ;
    std::cout << spacing[0] << " " << spacing[1] << " " << spacing[2] <<std::endl ;
    std::cout << origin[0] << " " << origin[1] << " " << origin[2] <<std::endl ;
    std::cout << centerImage[0] << " " << centerImage[1] << " " << centerImage[2] <<std::endl ;

    double d = camera->GetDistance() ;
    camera->SetFocalPoint ( centerImage ) ;
    camera->SetPosition ( centerImage[0], centerImage[1], d ) ;
    camera->ParallelProjectionOn () ;
    camera->SetParallelScale ( centerImage[2] ) ;

    // create window
    vtkSmartPointer < vtkRenderWindow > renderWindow = vtkSmartPointer < vtkRenderWindow >::New() ;
    renderWindow->SetSize ( 500, 500 ) ;
    renderWindow->AddRenderer ( renderer ) ;

    // create the interactor
    vtkSmartPointer < vtkRenderWindowInteractor > renderWindowInteractor = vtkSmartPointer < vtkRenderWindowInteractor > ::New() ;
    renderWindowInteractor->SetRenderWindow ( renderWindow ) ;

    // create a point picker
    vtkSmartPointer < vtkPointPicker > pointPicker = vtkSmartPointer < vtkPointPicker >::New() ;
    renderWindowInteractor->SetPicker ( pointPicker ) ;

    // set up the interactor style so we're slicing through the image rather than rotating around
    // vtkSmartPointer < vtkInteractorStyleImage > style = vtkSmartPointer < vtkInteractorStyleImage >::New() ;
    vtkSmartPointer < MyMouseCallbackInteractorStyle > style = vtkSmartPointer < MyMouseCallbackInteractorStyle >::New() ;
    // style->SetInteractionModeToImageSlicing () ;
    style->SetInteractionModeToImage3D();
    // style->SetInteractionModeToImage2D();
    renderWindowInteractor->SetInteractorStyle ( style ) ;

    vtkSmartPointer<vtkAxesActor> axes =
            vtkSmartPointer<vtkAxesActor>::New();

    vtkSmartPointer<vtkOrientationMarkerWidget> widget =
            vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    widget->SetOutlineColor( 0.900, 0.5500, 0.1000 );
    widget->SetOrientationMarker( axes );
    widget->SetInteractor( renderWindowInteractor );
    widget->SetViewport( 0.0, 0.0, 0.4, 0.4 );
    widget->SetEnabled( 1 );
    widget->InteractiveOn();

    renderWindowInteractor->Initialize();
    renderWindowInteractor->CreateRepeatingTimer(0.3);

    renderWindowInteractor->Start() ;
    return 0 ;

}

vtkVector3d get_next_point(vtkVector3d cur_point, vtkSmartPointer <vtkImageData> im, double step_size)
{
    vtkIdType voxel_id = im->FindPoint(cur_point.GetX(), cur_point.GetY(), cur_point.GetZ());
    double center_voxel_index[3];
    im->GetPoint(voxel_id, center_voxel_index);
    std::vector<vtkVector3d> neighbours;
    neighbours.push_back(vtkVector3d(center_voxel_index[0] + 1, center_voxel_index[1]    , center_voxel_index[2]    ));
    neighbours.push_back(vtkVector3d(center_voxel_index[0] - 1, center_voxel_index[1]    , center_voxel_index[2]    ));
    neighbours.push_back(vtkVector3d(center_voxel_index[0]    , center_voxel_index[1] + 1, center_voxel_index[2]    ));
    neighbours.push_back(vtkVector3d(center_voxel_index[0]    , center_voxel_index[1] - 1, center_voxel_index[2]    ));
    neighbours.push_back(vtkVector3d(center_voxel_index[0]    , center_voxel_index[1]    , center_voxel_index[2] + 1));
    neighbours.push_back(vtkVector3d(center_voxel_index[0]    , center_voxel_index[1]    , center_voxel_index[2] - 1));
    double min_dist = std::numeric_limits<double>::max();
    int min_index = 0;
    for (int i = 0; i < (int) neighbours.size(); i ++)
    {
        vtkVector3d cur_voxel_point = neighbours[i];
        vtkVector3d dist_vector = vtkVector3d(cur_voxel_point[0] - cur_point[0],cur_voxel_point[1] - cur_point[1],cur_voxel_point[2] - cur_point[2]);
        double cur_dist = dist_vector.SquaredNorm();
        if (cur_dist < min_dist)
        {
            min_dist = cur_dist;
            min_index = i;
        }
    }

    vtkVector3d min_dist_voxel = neighbours[min_index];
    vtkVector3d min_dist_voxel_dist_vect = vtkVector3d (min_dist_voxel[0] - cur_point[0],min_dist_voxel[1] - cur_point[1],min_dist_voxel[2] - cur_point[2]);
    double dist_to_min_dist_voxel = min_dist_voxel_dist_vect.SquaredNorm();
    vtkVector3d center_voxel_dist_vect = vtkVector3d (center_voxel_index[0] - cur_point[0],center_voxel_index[1] - cur_point[1],center_voxel_index[2] - cur_point[2]);
    double dist_to_center_voxel = center_voxel_dist_vect.SquaredNorm();
    double weight_min_dist_voxel = dist_to_center_voxel / (dist_to_min_dist_voxel + dist_to_center_voxel);
    double weigth_center_voxel = dist_to_min_dist_voxel / (dist_to_min_dist_voxel + dist_to_center_voxel);
    vtkVector3d direct_center_voxel = vtkVector3d(
            im->GetScalarComponentAsDouble(center_voxel_index[0], center_voxel_index[1], center_voxel_index[2], 0),
            im->GetScalarComponentAsDouble(center_voxel_index[0], center_voxel_index[1], center_voxel_index[2], 1),
            im->GetScalarComponentAsDouble(center_voxel_index[0], center_voxel_index[1], center_voxel_index[2], 2)
    );
    double direct_len_center_voxel = direct_center_voxel.Normalize();
    vtkVector3d direct_min_dist_voxel = vtkVector3d(
            im->GetScalarComponentAsDouble(min_dist_voxel[0], min_dist_voxel[1], min_dist_voxel[2], 0),
            im->GetScalarComponentAsDouble(min_dist_voxel[0], min_dist_voxel[1], min_dist_voxel[2], 1),
            im->GetScalarComponentAsDouble(min_dist_voxel[0], min_dist_voxel[1], min_dist_voxel[2], 2)
    );
    double direct_len_min_dist_voxel = direct_min_dist_voxel.Normalize();
    vtkVector3d direct_averaged = vtkVector3d(
            weight_min_dist_voxel * direct_min_dist_voxel[0] + weigth_center_voxel * direct_center_voxel[0],
            weight_min_dist_voxel * direct_min_dist_voxel[1] + weigth_center_voxel * direct_center_voxel[1],
            weight_min_dist_voxel * direct_min_dist_voxel[2] + weigth_center_voxel * direct_center_voxel[2]
    );
    vtkVector3d next_point = vtkVector3d(
            direct_averaged[0] * step_size + cur_point[0],
            direct_averaged[1] * step_size + cur_point[1],
            direct_averaged[2] * step_size + cur_point[2]
    );

    return next_point;
}

vtkSmartPointer <vtkPolyData> create_line_poly_data(const std::vector<vtkVector3d> & end_point_list, unsigned char color_rgb [])
{
    vtkSmartPointer<vtkPolyData> linePolyData = vtkSmartPointer<vtkPolyData>::New();

    vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
    for (size_t ip = 0; ip < end_point_list.size(); ip ++)
    {
        vtkVector3d cur_pt = end_point_list[ip];
        pts->InsertNextPoint(cur_pt[0], cur_pt[1], cur_pt[2]);
    }

    linePolyData->SetPoints(pts);

    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    for (size_t iLine = 0; iLine < pts->GetNumberOfPoints() - 1; iLine ++)
    {
        vtkSmartPointer<vtkLine> cur_line = vtkSmartPointer<vtkLine>::New();
        cur_line->GetPointIds()->SetId(0, iLine);
        cur_line->GetPointIds()->SetId(1, iLine + 1);
        lines->InsertNextCell(cur_line);
    }

    linePolyData->SetLines(lines);

    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    for (int iCell = 0; iCell < lines->GetNumberOfCells(); iCell++)
    {
        colors->InsertNextTupleValue(color_rgb);
    }

    linePolyData->GetCellData()->SetScalars(colors);

    return linePolyData;
}

class TimerCallback : public vtkCommand
{
public:
    TimerCallback ()
    {
    }

    ~TimerCallback()
    {
    }

    static TimerCallback *New()
    {
        TimerCallback *callback = new TimerCallback() ;
        return callback ;
    }

    virtual void Execute ( vtkObject *caller, unsigned long eventId, void *callData )
    {
        if (_cur_pt_id < 1000)
        {
            vtkSmartPointer<vtkCellArray> lines = this->_trajectory_line_vis->GetLines();
            vtkSmartPointer<vtkLine> cur_line = vtkSmartPointer<vtkLine>::New();
            cur_line->GetPointIds()->SetId(0, _cur_pt_id);
            cur_line->GetPointIds()->SetId(1, _cur_pt_id + 1);
            lines->InsertNextCell(cur_line);
            vtkSmartPointer<vtkUnsignedCharArray> colors = dynamic_cast<vtkUnsignedCharArray*>(this->_trajectory_line_vis->GetCellData()->GetScalars());
            colors->InsertNextTupleValue(this->_color_rgb);
            this->_trajectory_line_vis->Modified();
        }
        _cur_pt_id++;
    }

    void SetLineData (vtkSmartPointer<vtkPolyData> trajectory_line_ori, vtkSmartPointer<vtkPolyData> trajectory_line_vis, unsigned char color_rgb [])
    {
        this->_color_rgb[0] = color_rgb[0];
        this->_color_rgb[1] = color_rgb[1];
        this->_color_rgb[2] = color_rgb[2];
        this->_trajectory_line_vis = trajectory_line_vis;
        this->_trajectory_line_ori = trajectory_line_ori;
    }

private:
    int _cur_pt_id = 0;
    unsigned char _color_rgb[3];
    vtkSmartPointer<vtkPolyData> _trajectory_line_vis;
    vtkSmartPointer<vtkPolyData> _trajectory_line_ori;
} ;


void render_trajectory_with_timer(vtkSmartPointer<vtkPolyData> trajectory_line, unsigned char color_rgb [])
{
    vtkSmartPointer<vtkPolyData> trajectory_line_vis = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
    trajectory_line->GetPoints()->DeepCopy(pts);
    trajectory_line_vis->SetPoints(pts);
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    trajectory_line_vis->SetLines(lines);
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    trajectory_line_vis->GetCellData()->SetScalars(colors);
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(trajectory_line_vis);
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    vtkSmartPointer < vtkRenderer > renderer = vtkSmartPointer < vtkRenderer >::New() ;
    renderer->AddActor(actor);
    renderer->SetBackground(1, 1, 1);
    vtkSmartPointer < vtkRenderWindow > renderWindow = vtkSmartPointer < vtkRenderWindow >::New() ;
    renderWindow->AddRenderer ( renderer ) ;
    vtkSmartPointer < vtkRenderWindowInteractor > interactor = vtkSmartPointer < vtkRenderWindowInteractor >::New() ;
    interactor->SetRenderWindow ( renderWindow ) ;
    interactor->Initialize();
    interactor->CreateRepeatingTimer(0.3);
    vtkSmartPointer <TimerCallback> cb = vtkSmartPointer<TimerCallback>::New();
    cb->SetLineData(trajectory_line, trajectory_line_vis, color_rgb);
    interactor->AddObserver(vtkCommand::TimerEvent, cb);
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
    vtkSmartPointer<vtkOrientationMarkerWidget> widget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    widget->SetOutlineColor( 0.900, 0.5500, 0.1000 );
    widget->SetOrientationMarker( axes );
    widget->SetInteractor( interactor );
    widget->SetViewport( 0.0, 0.0, 0.4, 0.4 );
    widget->SetEnabled( 1 );
    widget->InteractiveOn();
    renderer->ResetCamera();
    renderWindow->Render();
    interactor->Start();
    return;
}
